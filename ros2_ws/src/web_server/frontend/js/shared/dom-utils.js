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
