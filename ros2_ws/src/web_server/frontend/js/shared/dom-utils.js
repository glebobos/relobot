/**
 * Safe DOM manipulation helpers to comply with secure coding practices and prevent XSS.
 */

import { AnsiUp } from 'ansi_up';

const ansiUp = new AnsiUp();

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
 * Safely parses text with ANSI escape sequences (colors, styles) using AnsiUp and appends it to a parent element.
 * Complies with strict XSS prevention guidelines by using DOMParser and DOM appending instead of innerHTML.
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

    // Set default message text color on container so unstyled child segments inherit it
    if (defaultMessageColor) {
        lineContainer.style.color = defaultMessageColor;
    }

    // 4. Parse ANSI escape sequences in the remaining text using AnsiUp
    const parsedHtml = ansiUp.ansi_to_html(textToProcess);
    
    // Safely parse the generated HTML using DOMParser to avoid direct innerHTML assignment
    const parser = new DOMParser();
    const doc = parser.parseFromString(parsedHtml, 'text/html');
    const nodes = Array.from(doc.body.childNodes);
    
    if (nodes.length > 0) {
        nodes.forEach(node => {
            lineContainer.appendChild(node);
        });
    } else {
        // Ensure empty lines still render a line break correctly
        lineContainer.appendChild(document.createElement('br'));
    }

    parentEl.appendChild(lineContainer);
}
