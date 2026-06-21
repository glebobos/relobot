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
        this.tabs.forEach((tab) => {
            tab.addEventListener('click', () => {
                const index = parseInt(tab.getAttribute('data-index'), 10);
                this.navigateToScreen(index);
            });
        });

        if (this.pipCamera) {
            this.pipCamera.addEventListener('click', () => this.navigateToScreen(1));
        }
        if (this.pipMap) {
            this.pipMap.addEventListener('click', () => this.navigateToScreen(0));
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
    }
}
