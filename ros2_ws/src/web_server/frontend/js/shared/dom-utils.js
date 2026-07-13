/**
 * Safe DOM manipulation helpers to comply with secure coding practices and prevent XSS.
 */

/**
 * Safely clears all children of a DOM element using replaceChildren.
 * @param {HTMLElement} element 
 */
export function safeClear(element) {
    if (element && typeof element.replaceChildren === 'function') {
        element.replaceChildren();
    } else if (element) {
        element.textContent = '';
    }
}

/**
 * Safely sets the text content of a DOM element.
 * @param {HTMLElement} element 
 * @param {string} text 
 */
export function safeSetText(element, text) {
    if (element) {
        element.textContent = text;
    }
}

/**
 * Safely creates a DOM element with classes, attributes, and text content.
 * @param {string} tag 
 * @param {string|string[]} classes 
 * @param {Object} attributes 
 * @param {string} text 
 * @returns {HTMLElement}
 */
export function safeCreateElement(tag, classes = [], attributes = {}, text = '') {
    const el = document.createElement(tag);
    
    // Add classes
    if (typeof classes === 'string' && classes.trim()) {
        el.className = classes;
    } else if (Array.isArray(classes)) {
        classes.forEach(cls => {
            if (cls) el.classList.add(cls);
        });
    }

    // Set attributes
    Object.entries(attributes).forEach(([key, val]) => {
        if (val !== undefined && val !== null) {
            el.setAttribute(key, String(val));
        }
    });

    // Set safe text
    if (text) {
        el.textContent = text;
    }

    return el;
}

/**
 * Safely parses text with ANSI escape sequences (colors, styles) and appends it to a parent element.
 * Complies with strict XSS prevention guidelines by using document.createElement and textContent.
 * @param {HTMLElement} parentEl 
 * @param {string} lineText 
 */
export function appendLogLine(parentEl, lineText) {
    if (!parentEl) return;

    const lineContainer = document.createElement('div');
    lineContainer.className = 'c-logs-line';

    // Strip trailing newlines since block-level div handles line breaks
    const cleanText = lineText.endsWith('\n') ? lineText.slice(0, -1) : lineText;

    let textToProcess = cleanText;

    // 1. Extract Timestamp [HH:MM:SS]
    const timeMatch = textToProcess.match(/^\[(\d{2}:\d{2}:\d{2})\]\s*/);
    if (timeMatch) {
        const timeSpan = document.createElement('span');
        timeSpan.textContent = timeMatch[0];
        timeSpan.style.color = '#64748b'; // Muted slate gray
        lineContainer.appendChild(timeSpan);
        textToProcess = textToProcess.slice(timeMatch[0].length);
    }

    let defaultMessageColor = null;

    // 2. Extract Level and Node: [LEVEL] [node_name]:
    const levelNodeMatch = textToProcess.match(/^\[(INFO|WARN|ERROR|FATAL|DEBUG)\]\s+\[([^\]]+)\]:\s*/i);
    if (levelNodeMatch) {
        const level = levelNodeMatch[1].toUpperCase();
        const node = levelNodeMatch[2];

        const levelSpan = document.createElement('span');
        levelSpan.textContent = `[${level}] `;
        
        const levelColors = {
            'DEBUG': '#94a3b8', // Gray
            'INFO': '#4ade80',  // Green
            'WARN': '#facc15',  // Yellow
            'ERROR': '#f87171', // Red
            'FATAL': '#f43f5e'  // Pink/Bright Red
        };
        levelSpan.style.color = levelColors[level] || '#4ade80';
        levelSpan.style.fontWeight = 'bold';
        lineContainer.appendChild(levelSpan);

        const nodeSpan = document.createElement('span');
        nodeSpan.textContent = `[${node}]: `;
        nodeSpan.style.color = '#38bdf8'; // Sky blue / cyan
        lineContainer.appendChild(nodeSpan);

        // Determine message-wide default color if it's warning or error
        if (level === 'WARN') {
            defaultMessageColor = '#facc15';
        } else if (level === 'ERROR' || level === 'FATAL') {
            defaultMessageColor = '#f87171';
        }

        textToProcess = textToProcess.slice(levelNodeMatch[0].length);
    } else {
        // 3. Fallback: Extract generic Tag: [Tag]
        const tagMatch = textToProcess.match(/^\[([^\]]+)\]\s*/);
        if (tagMatch) {
            const tagSpan = document.createElement('span');
            tagSpan.textContent = tagMatch[0];
            tagSpan.style.color = '#38bdf8'; // Cyan
            lineContainer.appendChild(tagSpan);
            textToProcess = textToProcess.slice(tagMatch[0].length);
        }
    }

    // 4. Parse ANSI escape sequences in the remaining text
    const ansiRegex = /\u001b\[([0-9;]*)m/g;
    let lastIndex = 0;
    let match;

    // Current state of style
    let currentStyle = {
        bold: false,
        italic: false,
        underline: false,
        fg: defaultMessageColor,
        bg: null
    };

    function applyStyles(span, style) {
        if (style.bold) span.style.fontWeight = 'bold';
        if (style.italic) span.style.fontStyle = 'italic';
        if (style.underline) span.style.textDecoration = 'underline';
        
        // Use styled fg if set, otherwise fall back to level-based defaultMessageColor
        const fgColor = style.fg || defaultMessageColor;
        if (fgColor) span.style.color = fgColor;
        
        if (style.bg) span.style.backgroundColor = style.bg;
    }

    const colorMap = {
        // Standard colors
        '30': '#1e293b', // Black
        '31': '#f87171', // Red
        '32': '#4ade80', // Green
        '33': '#facc15', // Yellow
        '34': '#60a5fa', // Blue
        '35': '#c084fc', // Magenta
        '36': '#2dd4bf', // Cyan
        '37': '#e2e8f0', // White

        // Bright colors
        '90': '#94a3b8',
        '91': '#fca5a5',
        '92': '#86efac',
        '93': '#fef08a',
        '94': '#93c5fd',
        '95': '#d8b4fe',
        '96': '#67e8f9',
        '97': '#ffffff',
    };

    const bgMap = {
        '40': '#0f172a',
        '41': '#991b1b',
        '42': '#166534',
        '43': '#854d0e',
        '44': '#1e40af',
        '45': '#6b21a8',
        '46': '#115e59',
        '47': '#334155',

        '100': '#1e293b',
        '101': '#b91c1c',
        '102': '#15803d',
        '103': '#a16207',
        '104': '#1d4ed8',
        '105': '#7e22ce',
        '106': '#0f766e',
        '107': '#475569',
    };

    while ((match = ansiRegex.exec(textToProcess)) !== null) {
        // Add preceding text segment with current styles
        const textSegment = textToProcess.slice(lastIndex, match.index);
        if (textSegment) {
            const span = document.createElement('span');
            span.textContent = textSegment;
            applyStyles(span, currentStyle);
            lineContainer.appendChild(span);
        }

        // Parse codes
        const codes = match[1].split(';');
        codes.forEach(codeStr => {
            const code = parseInt(codeStr, 10) || 0;
            if (code === 0) {
                // Reset
                currentStyle = { bold: false, italic: false, underline: false, fg: defaultMessageColor, bg: null };
            } else if (code === 1) {
                currentStyle.bold = true;
            } else if (code === 3) {
                currentStyle.italic = true;
            } else if (code === 4) {
                currentStyle.underline = true;
            } else if (code >= 30 && code <= 37) {
                currentStyle.fg = colorMap[code.toString()];
            } else if (code === 39) {
                currentStyle.fg = defaultMessageColor; // Reset foreground to default
            } else if (code >= 40 && code <= 47) {
                currentStyle.bg = bgMap[code.toString()];
            } else if (code === 49) {
                currentStyle.bg = null; // Reset background
            } else if (code >= 90 && code <= 97) {
                currentStyle.fg = colorMap[code.toString()];
            } else if (code >= 100 && code <= 107) {
                currentStyle.bg = bgMap[code.toString()];
            }
        });

        lastIndex = ansiRegex.lastIndex;
    }

    // Add remaining text
    const remainingText = textToProcess.slice(lastIndex);
    if (remainingText) {
        const span = document.createElement('span');
        span.textContent = remainingText;
        applyStyles(span, currentStyle);
        lineContainer.appendChild(span);
    } else if (lineContainer.childNodes.length === 0) {
        // Ensure empty lines still render a line break correctly
        lineContainer.appendChild(document.createElement('br'));
    }

    parentEl.appendChild(lineContainer);
}
