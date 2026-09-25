export class NavigationManager {
    constructor(screensWrapperId, navTabsSelector, mainCameraService, pipCameraService) {
        this.screensWrapper = document.getElementById(screensWrapperId);
        this.tabs = document.querySelectorAll(navTabsSelector);
        this.mainCameraService = mainCameraService;
        this.pipCameraService = pipCameraService;
        this.currentScreen = 0;

        this.pipCamera = document.getElementById('pipCamera');
        this.pipMap = document.getElementById('pipMap');
        this.mapVideoToggleBtn = document.getElementById('map-video-toggle-btn');
        this.isMapVideoEnabled = localStorage.getItem('map_video_enabled') !== 'false';
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

        if (this.mapVideoToggleBtn) {
            this.mapVideoToggleHandler = () => this.toggleMapVideo();
            this.mapVideoToggleBtn.addEventListener('click', this.mapVideoToggleHandler);
        }

        this.updateMapVideoUI();

        // Initialize streams and view based on active screen
        this.navigateToScreen(0);
    }

    toggleMapVideo(forceState) {
        if (forceState !== undefined) {
            this.isMapVideoEnabled = Boolean(forceState);
        } else {
            this.isMapVideoEnabled = !this.isMapVideoEnabled;
        }
        localStorage.setItem('map_video_enabled', String(this.isMapVideoEnabled));
        this.updateMapVideoUI();

        if (this.currentScreen === 0) {
            if (this.isMapVideoEnabled) {
                if (this.pipCameraService) this.pipCameraService.connect();
            } else {
                if (this.pipCameraService) this.pipCameraService.stop();
            }
        }
    }

    updateMapVideoUI() {
        if (this.pipCamera) {
            this.pipCamera.style.display = this.isMapVideoEnabled ? '' : 'none';
        }
        if (this.mapVideoToggleBtn) {
            this.mapVideoToggleBtn.classList.toggle('is-disabled', !this.isMapVideoEnabled);
            const icon = this.mapVideoToggleBtn.querySelector('i');
            if (icon) {
                icon.className = this.isMapVideoEnabled ? 'fas fa-video' : 'fas fa-video-slash';
            }
            this.mapVideoToggleBtn.title = this.isMapVideoEnabled ? 'Disable Video Feed' : 'Enable Video Feed';
        }
    }

    navigateToScreen(index) {
        if (index < 0 || index > 3) return;
        this.currentScreen = index;
        if (this.screensWrapper) {
            this.screensWrapper.style.transform = `translateX(-${index * 25}%)`;
        }
        this.tabs.forEach((tab, idx) => {
            tab.classList.toggle('is-active', idx === index);
        });

        // Optimize camera streams based on active screen
        if (index === 0) {
            if (this.pipCameraService && this.isMapVideoEnabled) this.pipCameraService.connect();
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
        if (this.mapVideoToggleBtn && this.mapVideoToggleHandler) {
            this.mapVideoToggleBtn.removeEventListener('click', this.mapVideoToggleHandler);
        }
    }
}
