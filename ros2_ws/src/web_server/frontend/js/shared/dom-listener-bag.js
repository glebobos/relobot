export class DomListenerBag {
    constructor() {
        this.listeners = [];
    }

    add(target, type, handler, options) {
        if (!target) return handler;
        target.addEventListener(type, handler, options);
        this.listeners.push({ target, type, handler, options });
        return handler;
    }

    destroy() {
        this.listeners.splice(0).reverse().forEach(({ target, type, handler, options }) => {
            target.removeEventListener(type, handler, options);
        });
    }
}
