//! Rosbridge v2 JSON protocol message framing and serialization utilities.

use serde::Deserialize;

/// Parsed client operations received over WebSocket.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClientOp<'a> {
    /// Subscribe to a ROS 2 topic.
    Subscribe {
        /// Target topic name (e.g. `"/tf"`).
        topic: &'a str,
        /// Optional subscription identifier.
        id: Option<&'a str>,
        /// Optional client throttle interval in milliseconds.
        throttle_rate_ms: u64,
    },
    /// Unsubscribe from a previously subscribed topic.
    Unsubscribe {
        /// Target topic name.
        topic: &'a str,
        /// Optional subscription identifier.
        id: Option<&'a str>,
    },
    /// Any other operation (e.g., publish, call_service) to be forwarded upstream.
    Other,
}

#[derive(Deserialize)]
struct RawClientMessage<'a> {
    op: Option<&'a str>,
    topic: Option<&'a str>,
    id: Option<&'a str>,
    #[serde(default)]
    throttle_rate: Option<u64>,
}

/// Parses an incoming rosbridge JSON payload into a structured `ClientOp`.
///
/// Uses zero-copy string slicing from the input JSON slice.
#[inline]
#[must_use]
pub fn parse_client_message(text: &str) -> ClientOp<'_> {
    if let Ok(raw) = serde_json::from_str::<RawClientMessage>(text) {
        match raw.op {
            Some("subscribe") => {
                let topic = raw.topic.unwrap_or_default();
                ClientOp::Subscribe {
                    topic,
                    id: raw.id,
                    throttle_rate_ms: raw.throttle_rate.unwrap_or(0),
                }
            }
            Some("unsubscribe") => {
                let topic = raw.topic.unwrap_or_default();
                ClientOp::Unsubscribe {
                    topic,
                    id: raw.id,
                }
            }
            _ => ClientOp::Other,
        }
    } else {
        ClientOp::Other
    }
}

/// Formats a rosbridge v2 JSON publish message envelope efficiently into a pre-allocated string.
///
/// Avoids repeated memory allocations by computing exact required buffer capacity upfront.
#[must_use]
pub fn format_rosbridge_publish(topic: &str, raw_msg_json: &str, id: Option<&str>) -> String {
    // Envelope format:
    // With id:    {"op":"publish","topic":"<topic>","id":"<id>","msg":<raw_msg_json>}
    // Without id: {"op":"publish","topic":"<topic>","msg":<raw_msg_json>}
    if let Some(sub_id) = id {
        let capacity = 40 + topic.len() + sub_id.len() + raw_msg_json.len();
        let mut out = String::with_capacity(capacity);
        out.push_str("{\"op\":\"publish\",\"topic\":\"");
        out.push_str(topic);
        out.push_str("\",\"id\":\"");
        out.push_str(sub_id);
        out.push_str("\",\"msg\":");
        out.push_str(raw_msg_json);
        out.push('}');
        out
    } else {
        let capacity = 30 + topic.len() + raw_msg_json.len();
        let mut out = String::with_capacity(capacity);
        out.push_str("{\"op\":\"publish\",\"topic\":\"");
        out.push_str(topic);
        out.push_str("\",\"msg\":");
        out.push_str(raw_msg_json);
        out.push('}');
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_publish_without_id() {
        let formatted = format_rosbridge_publish("/map", "{\"data\":[1,2]}", None);
        assert_eq!(
            formatted,
            "{\"op\":\"publish\",\"topic\":\"/map\",\"msg\":{\"data\":[1,2]}}"
        );
    }

    #[test]
    fn test_format_publish_with_id() {
        let formatted = format_rosbridge_publish("/tf", "{\"transforms\":[]}", Some("sub_1"));
        assert_eq!(
            formatted,
            "{\"op\":\"publish\",\"topic\":\"/tf\",\"id\":\"sub_1\",\"msg\":{\"transforms\":[]}}"
        );
    }

    #[test]
    fn test_format_publish_nested_and_array_json() {
        let raw = "{\"info\":{\"width\":100,\"height\":100},\"data\":[-1,0,100]}";
        let formatted = format_rosbridge_publish("/map", raw, Some("map_sub_42"));
        assert_eq!(
            formatted,
            "{\"op\":\"publish\",\"topic\":\"/map\",\"id\":\"map_sub_42\",\"msg\":{\"info\":{\"width\":100,\"height\":100},\"data\":[-1,0,100]}}"
        );
    }

    #[test]
    fn test_parse_client_message_subscribe() {
        let json = r#"{"op":"subscribe","topic":"/tf","id":"sub_tf_1","throttle_rate":50}"#;
        assert_eq!(
            parse_client_message(json),
            ClientOp::Subscribe {
                topic: "/tf",
                id: Some("sub_tf_1"),
                throttle_rate_ms: 50,
            }
        );
    }

    #[test]
    fn test_parse_client_message_unsubscribe() {
        let json = r#"{"op":"unsubscribe","topic":"/robot_pose","id":"pose_sub"}"#;
        assert_eq!(
            parse_client_message(json),
            ClientOp::Unsubscribe {
                topic: "/robot_pose",
                id: Some("pose_sub"),
            }
        );
    }

    #[test]
    fn test_parse_client_message_other() {
        let json = r#"{"op":"call_service","service":"/set_pose","args":{}}"#;
        assert_eq!(parse_client_message(json), ClientOp::Other);

        let invalid_json = "not valid json";
        assert_eq!(parse_client_message(invalid_json), ClientOp::Other);
    }
}
