import Swal from 'sweetalert2';

/**
 * Show a custom, thread-safe, non-blocking confirmation modal using SweetAlert2.
 * @param {string} message 
 * @param {Function} onConfirm Callback when "Yes" is clicked 
 */
export function showConfirm(message, onConfirm) {
    Swal.fire({
        text: message,
        icon: 'question',
        showCancelButton: true,
        confirmButtonText: 'Yes',
        cancelButtonText: 'No',
        background: '#131822',
        color: '#f8fafc',
        confirmButtonColor: '#10b981',
        cancelButtonColor: '#ef4444',
        customClass: {
            popup: 'c-swal-popup',
            confirmButton: 'c-swal-confirm',
            cancelButton: 'c-swal-cancel'
        }
    }).then((result) => {
        if (result.isConfirmed && typeof onConfirm === 'function') {
            onConfirm();
        }
    });
}

/**
 * Show a custom, thread-safe, non-blocking alert modal using SweetAlert2.
 * @param {string} message 
 */
export function showAlert(message) {
    Swal.fire({
        text: message,
        icon: 'info',
        confirmButtonText: 'OK',
        background: '#131822',
        color: '#f8fafc',
        confirmButtonColor: '#10b981',
        customClass: {
            popup: 'c-swal-popup',
            confirmButton: 'c-swal-confirm'
        }
    });
}
