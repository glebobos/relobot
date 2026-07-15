export class NavigationManager {
    constructor(screensWrapperId, navTabsSelector, mainCameraService, pipCameraService) {
        this.screensWrapper = document.getElementById(screensWrapperId);
        this.tabs = document.querySelectorAll(navTabsSelector);
        this.mainCameraService = mainCameraService;
        this.pipCameraService = pipCameraService;
        this.currentScreen = 0;

        this.pipCamera = document.getElementById('pipCamera');
        this.pipMap = document.getElementById('pipMap');
    }

    init() {
        this.tabHandlers = new Map();
        this.tabs.forEach((tab) => {
            const handler = () => {
                const index = parseInt(tab.getAttribute('data-index'), 10);
                this.navigateToScreen(index);
            };
            this.tabHandlers.set(tab, handler);
            tab.addEventListener('click', handler);
        });

        if (this.pipCamera) {
            this.pipCameraHandler = () => this.navigateToScreen(1);
            this.pipCamera.addEventListener('click', this.pipCameraHandler);
        }
        if (this.pipMap) {
            this.pipMapHandler = () => this.navigateToScreen(0);
            this.pipMap.addEventListener('click', this.pipMapHandler);
        }

        // Initialize streams and view based on active screen
        this.navigateToScreen(0);
    }

    navigateToScreen(index) {
        if (index < 0 || index > 2) return;
        this.currentScreen = index;
        if (this.screensWrapper) {
            this.screensWrapper.style.transform = `translateX(-${index * 33.3333}%)`;
        }
        this.tabs.forEach((tab, idx) => {
            tab.classList.toggle('is-active', idx === index);
        });

        // Optimize camera streams based on active screen
        if (index === 0) {
            if (this.pipCameraService) this.pipCameraService.connect();
            if (this.mainCameraService) this.mainCameraService.stop();
        } else if (index === 1) {
            if (this.mainCameraService) this.mainCameraService.connect();
            if (this.pipCameraService) this.pipCameraService.stop();
        } else {
            if (this.mainCameraService) this.mainCameraService.stop();
            if (this.pipCameraService) this.pipCameraService.stop();
        }

        // Dispatch custom event to notify other components of screen change
        window.dispatchEvent(new CustomEvent('screenChanged', { detail: { index } }));
    }

    destroy() {
        if (this.tabHandlers) {
            this.tabHandlers.forEach((handler, tab) => tab.removeEventListener('click', handler));
            this.tabHandlers.clear();
        }
        if (this.pipCamera && this.pipCameraHandler) {
            this.pipCamera.removeEventListener('click', this.pipCameraHandler);
        }
        if (this.pipMap && this.pipMapHandler) {
            this.pipMap.removeEventListener('click', this.pipMapHandler);
        }
    }
}
