# BEM Migration Mapping

This document provides a translation table between the legacy CSS classes and the new BEM classes. These changes will be applied atomically per component across HTML, CSS, and JS files.

## State classes (SMACSS/ITCSS Standard)
We map general state classes to the `is-*` utility convention:
- `.active` -> `.is-active`
- `.open` -> `.is-open`
- `.dropdown-open` -> `.is-dropdown-open`
- `.docking-active` -> `.is-docking-active`
- `.toast-hidden` -> `.is-toast-hidden`
- `.warning` -> `.is-warning`

---

## Structure & Layout Objects (`o-` prefix)
| Legacy CSS Class | New BEM Object Class | Notes |
| :--- | :--- | :--- |
| `.app-container` | `.o-app-container` | Root viewport container |
| `.screens-viewport` | `.o-screens-viewport` | Horizontal viewport mask |
| `.screens-wrapper` | `.o-screens-wrapper` | Horizontal slider canvas |
| `.screen` | `.o-screen` | Screen layout |

---

## Global Components (`c-` prefix)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.horizontal-slider` | `.c-slider` | Custom range slider component |
| `.sidebar-btn` | `.c-sidebar-btn` | Floating round button component |
| `.stop-btn-wide` | `.c-stop-btn-wide` | Wide emergency stop button |

---

## Header Component (`c-app-header`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.app-header` | `.c-app-header` | Top navigation bar |
| `.app-logo` | `.c-app-header__logo` | Logo and title |
| `.header-nav` | `.c-app-header__nav` | Top navigation indicators |
| `.header-right` | `.c-app-header__right` | Header status section |
| `.header-time` | `.c-app-header__time` | Time indicator |

---

## RPM Widget Component (`c-rpm-widget`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.header-rpm` | `.c-rpm-widget` | Header RPM display widget |
| `.rpm-collapsed-info` | `.c-rpm-widget__collapsed-info` | RPM value wrapper |
| `.rpm-unit` | `.c-rpm-widget__unit` | RPM unit label |
| `.knives-slider-inline` | `.c-rpm-widget__slider-dropdown` | Header RPM slider |
| `.knives-speed-label` | `.c-rpm-widget__speed-label` | Blade speed text |

---

## Battery & Docking Widget (`c-battery-widget`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.header-battery` | `.c-battery-widget` | Battery status |
| `.battery-dock-dropdown` | `.c-battery-widget__dock-dropdown` | Dock/Undock popup panel |
| `.dock-dropdown-btn` | `.c-battery-widget__dock-btn` | Action button inside dropdown |

---

## Map View Component (`c-map-view`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.map-view-container` | `.c-map-view` | Map container block |
| `.map-sidebar` | `.c-map-sidebar` | Overlay toolbar |
| `.map-top-controls` | `.c-map-view__top-controls` | Floating top row |
| `.map-bottom-controls` | `.c-map-view__bottom-controls` | Floating bottom row |
| `.map-action-buttons` | `.c-map-view__action-buttons` | Floating action buttons |
| `.fui-dock-panel` | `.c-fui-dock-panel` | Docking status panel |
| `.fui-icon` | `.c-fui-dock-panel__icon` | Icon |
| `.fui-text` | `.c-fui-dock-panel__text` | Label and state wrapper |
| `.fui-title` | `.c-fui-dock-panel__title` | Title |
| `.fui-sub` | `.c-fui-dock-panel__sub` | Subtext description |

---

## Camera View Component (`c-camera-view`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.camera-view-container` | `.c-camera-view` | Camera wrapper block |
| `.video-wrapper` | `.c-camera-view__video-wrapper` | Camera stream container |
| `.camera-stop-btn` | `.c-camera-view__stop-btn` | Emergency stop button (Legacy) |
| `.camera-bottom-controls` | `.c-camera-view__bottom-controls` | Layout block for bottom actions |
| `.knives-slider-container` | `.c-camera-view__slider-container`| Speed slider block |

---

## Picture-in-Picture Component (`c-pip-overlay`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.pip-overlay` | `.c-pip-overlay` | PIP wrapper |
| `.camera-pip` | `.c-pip-overlay--camera` | Camera PIP modifier |
| `.map-pip` | `.c-pip-overlay--map` | Map PIP modifier |
| `.pip-label` | `.c-pip-overlay__label` | Indicator badge label |

---

## Settings Component (`c-settings-panel`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.settings-container` | `.c-settings-panel` | Settings panel block |
| `.settings-title` | `.c-settings-panel__title` | Title |
| `.settings-list` | `.c-settings-panel__list` | List items wrapper |
| `.settings-item` | `.c-settings-item` | Single list item |
| `.settings-item-left` | `.c-settings-item__left` | Alignment |
| `.settings-icon` | `.c-settings-item__icon` | Category icon |
| `.settings-chevron` | `.c-settings-item__chevron` | Navigation chevron |

---

## Settings Drawer Component (`c-settings-drawer`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.settings-drawer` | `.c-settings-drawer` | Drawer overlay panel |
| `.drawer-header` | `.c-settings-drawer__header` | Header row |
| `.drawer-back-btn` | `.c-settings-drawer__back-btn` | Dismiss button |
| `.drawer-title` | `.c-settings-drawer__title` | Active section title |
| `.drawer-content` | `.c-settings-drawer__content` | Form content container |
| `.drawer-row` | `.c-settings-drawer__row` | Flex list item |
| `.drawer-row-label` | `.c-settings-drawer__row-label` | Field description |
| `.drawer-row-val` | `.c-settings-drawer__row-val` | Field value |
| `.drawer-btn` | `.c-settings-drawer__btn` | Button |
| `.drawer-btn-primary` | `.c-settings-drawer__btn--primary` | Modifier primary button |
| `.drawer-btn-danger` | `.c-settings-drawer__btn--danger` | Modifier danger button |
| `.drawer-btn-warning` | `.c-settings-drawer__btn--warning` | Modifier warning button |
| `.logs-console` | `.c-logs-console` | Custom logging terminal |

---

## Modal Component (`c-modal`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.modal-backdrop` | `.c-modal-backdrop` | Backdrop shadow |
| `.modal-content` | `.c-modal-content` | Content card |
| `.modal-buttons` | `.c-modal-content__buttons` | Button row |
| `.modal-btn` | `.c-modal-content__btn` | Common button |
| `.modal-btn-yes` | `.c-modal-content__btn--yes` | Modifier yes button |
| `.modal-btn-no` | `.c-modal-content__btn--no` | Modifier no button |
| `.modal-btn-ok` | `.c-modal-content__btn--ok` | Modifier ok button |

---

## Dots Navigation Component (`c-dots-nav`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.dots-nav` | `.c-dots-nav` | Indicator wrapper |
| `.dot` | `.c-dots-nav__dot` | Dot indicator |

---

## Virtual Joystick Component (`c-joystick`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `#vj-overlay` | `.c-joystick-overlay` | Floating window (converted to class) |
| `#vj-base` | `.c-joystick-base` | Outer base ring (converted to class) |
| `#vj-knob` | `.c-joystick-knob` | Center handle knob (converted to class) |

---

## Toast Notification Component (`c-coverage-toast`)
| Legacy CSS Class | New BEM Component Class | Notes |
| :--- | :--- | :--- |
| `.coverage-toast` | `.c-coverage-toast` | Floating toast alert |
