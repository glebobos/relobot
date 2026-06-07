export class Clock {
    constructor(elementId) {
        this.statusTime = document.getElementById(elementId);
        this.intervalId = null;
    }

    start() {
        if (!this.statusTime) return;
        const updateClock = () => {
            const now = new Date();
            const hrs = String(now.getHours()).padStart(2, '0');
            const mins = String(now.getMinutes()).padStart(2, '0');
            this.statusTime.textContent = `${hrs}:${mins}`;
        };
        updateClock();
        this.intervalId = setInterval(updateClock, 1000);
    }

    stop() {
        if (this.intervalId) {
            clearInterval(this.intervalId);
            this.intervalId = null;
        }
    }
}
