import { rosService } from '../../services/ros-service.js';
import { CMDS, MSG_TYPES, SERVICES } from '../../shared/constants.js';
import { DomListenerBag } from '../../shared/dom-listener-bag.js';
import { showAlert, showConfirm } from '../modal.js';

export class SystemOperationsController {
    constructor(sendCoverageCommand, sendSystemCommand) {
        this.sendCoverageCommand = sendCoverageCommand;
        this.sendSystemCommand = sendSystemCommand;
        this.listeners = new DomListenerBag();
        this.initialize();
    }

    initialize() {
        this.bindConfirmed('save-map-btn', 'Save current map? This will overwrite the previous save.', () => {
            const client = rosService.createServiceV2(
                SERVICES.SERIALIZE_MAP,
                MSG_TYPES.SERIALIZE_MAP_SRV,
            );
            client.callService(
                { filename: 'map_serialized' },
                () => showAlert('Map saved!'),
                error => showAlert(`Error saving map: ${error}`),
            );
        });
        this.bindConfirmed(
            'reset-map-btn',
            'Reset map? This will delete the saved map and restart SLAM in mapping mode.',
            () => {
                this.sendCoverageCommand(CMDS.RESET_MAP);
                showAlert('Resetting map and restarting SLAM...');
            },
        );
        this.bindConfirmed(
            'restart-slam-btn',
            'Restart SLAM toolbox? This will reload the saved map if it exists.',
            () => {
                this.sendCoverageCommand(CMDS.RESTART_SLAM);
                showAlert('Restarting SLAM...');
            },
        );
        this.bindConfirmed('reboot-pi-btn', 'Are you sure you want to reboot the Relobot?', () => {
            this.sendSystemCommand(CMDS.REBOOT);
            showAlert('Rebooting Relobot...');
        });
        this.bindConfirmed('poweroff-pi-btn', 'Are you sure you want to power off the Relobot?', () => {
            this.sendSystemCommand(CMDS.POWEROFF);
            showAlert('Powering off Relobot...');
        });
    }

    bindConfirmed(elementId, message, action) {
        const element = document.getElementById(elementId);
        this.listeners.add(element, 'click', () => showConfirm(message, action));
    }

    destroy() {
        this.listeners.destroy();
        this.sendCoverageCommand = null;
        this.sendSystemCommand = null;
    }
}
