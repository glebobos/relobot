/**
 * Show a custom, thread-safe, non-blocking confirmation modal.
 * @param {string} message 
 * @param {Function} onConfirm Callback when "Yes" is clicked 
 */
export function showConfirm(message, onConfirm) {
    const backdrop = document.createElement('div');
    backdrop.className = 'c-modal-backdrop';

    const modal = document.createElement('div');
    modal.className = 'c-modal-content';

    const text = document.createElement('p');
    text.textContent = message;
    modal.appendChild(text);

    const btnContainer = document.createElement('div');
    btnContainer.className = 'c-modal-content__buttons';

    const yesBtn = document.createElement('button');
    yesBtn.textContent = 'Yes';
    yesBtn.className = 'c-modal-content__btn c-modal-content__btn--yes';
    yesBtn.addEventListener('click', () => {
        backdrop.remove();
        if (typeof onConfirm === 'function') {
            onConfirm();
        }
    });

    const noBtn = document.createElement('button');
    noBtn.textContent = 'No';
    noBtn.className = 'c-modal-content__btn c-modal-content__btn--no';
    noBtn.addEventListener('click', () => {
        backdrop.remove();
    });

    btnContainer.appendChild(yesBtn);
    btnContainer.appendChild(noBtn);
    modal.appendChild(btnContainer);
    backdrop.appendChild(modal);
    document.body.appendChild(backdrop);
}

/**
 * Show a custom, thread-safe, non-blocking alert modal.
 * @param {string} message 
 */
export function showAlert(message) {
    const backdrop = document.createElement('div');
    backdrop.className = 'c-modal-backdrop';

    const modal = document.createElement('div');
    modal.className = 'c-modal-content';

    const text = document.createElement('p');
    text.textContent = message;
    modal.appendChild(text);

    const btnContainer = document.createElement('div');
    btnContainer.className = 'c-modal-content__buttons';

    const okBtn = document.createElement('button');
    okBtn.textContent = 'OK';
    okBtn.className = 'c-modal-content__btn c-modal-content__btn--ok';
    okBtn.addEventListener('click', () => {
        backdrop.remove();
    });

    btnContainer.appendChild(okBtn);
    modal.appendChild(btnContainer);
    backdrop.appendChild(modal);
    document.body.appendChild(backdrop);
}
