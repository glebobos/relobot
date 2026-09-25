/**
 * ReloBot Presentation Deck Engine
 * Handles slide navigation, keyboard shortcuts, speaker notes drawer, fullscreen, and stopwatch.
 */

(function () {
  let currentSlide = 1;
  const slides = document.querySelectorAll('.slide');
  const totalSlides = slides.length || 6;

  const progressBar = document.getElementById('progressBar');
  const progressTrack = document.querySelector('.progress-track');
  const slideCounter = document.getElementById('slideCounter');
  const prevBtn = document.getElementById('prevBtn');
  const nextBtn = document.getElementById('nextBtn');
  const speakerDrawer = document.getElementById('speakerDrawer');
  const speakerNotesText = document.getElementById('speakerNotesText');
  const notesSlideIndicator = document.getElementById('notesSlideIndicator');
  const toggleNotesBtn = document.getElementById('toggleNotesBtn');
  const closeNotesBtn = document.getElementById('closeNotesBtn');
  const fullscreenBtn = document.getElementById('fullscreenBtn');
  const timerBadge = document.getElementById('timerBadge');
  const timerText = document.getElementById('timerText');
  const slideCanvas = document.getElementById('slideCanvas');
  const deckStage = document.querySelector('.deck-stage');

  function fitSlides() {
    if (!slideCanvas || !deckStage) return;
    const scale = Math.min(deckStage.clientWidth / 1440, deckStage.clientHeight / 810);
    slideCanvas.style.width = '1440px';
    slideCanvas.style.height = '810px';
    slideCanvas.style.transformOrigin = 'top left';
    slideCanvas.style.transform = `scale(${scale})`;
  }

  window.addEventListener('resize', fitSlides);
  fitSlides();

  function updateSlide(index) {
    if (index < 1 || index > totalSlides) return;
    currentSlide = index;

    slides.forEach((slide) => {
      const slideIdx = parseInt(slide.getAttribute('data-index'), 10);
      if (slideIdx === currentSlide) {
        slide.classList.add('active');
        slide.setAttribute('aria-hidden', 'false');
      } else {
        slide.classList.remove('active');
        slide.setAttribute('aria-hidden', 'true');
      }
    });

    // Progress bar & counter
    const progress = (currentSlide / totalSlides) * 100;
    if (progressBar) progressBar.style.width = `${progress}%`;
    if (progressTrack) progressTrack.setAttribute('aria-valuenow', currentSlide);
    if (slideCounter) {
      slideCounter.textContent = `${String(currentSlide).padStart(2, '0')} / ${String(totalSlides).padStart(2, '0')}`;
    }

    // Prev/Next buttons state
    if (prevBtn) prevBtn.disabled = currentSlide === 1;
    if (nextBtn) nextBtn.disabled = currentSlide === totalSlides;

    // Update speaker notes
    if (typeof speakerNotes !== 'undefined' && speakerNotesText) {
      speakerNotesText.textContent = speakerNotes[currentSlide] || "";
    }
    if (notesSlideIndicator) {
      notesSlideIndicator.textContent = `Slide ${currentSlide} / ${totalSlides}`;
    }
  }

  function nextSlide() {
    if (currentSlide < totalSlides) updateSlide(currentSlide + 1);
  }

  function prevSlide() {
    if (currentSlide > 1) updateSlide(currentSlide - 1);
  }

  function toggleNotes() {
    if (!speakerDrawer) return;
    speakerDrawer.classList.toggle('open');
    const isOpen = speakerDrawer.classList.contains('open');
    speakerDrawer.setAttribute('aria-hidden', String(!isOpen));
    if (toggleNotesBtn) {
      toggleNotesBtn.classList.toggle('active', isOpen);
      toggleNotesBtn.setAttribute('aria-expanded', String(isOpen));
    }
  }

  function toggleFullscreen() {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen().catch(() => {});
    } else {
      document.exitFullscreen().catch(() => {});
    }
  }

  // Keyboard navigation
  window.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowRight' || e.key === ' ' || e.key === 'PageDown') {
      e.preventDefault();
      nextSlide();
    } else if (e.key === 'ArrowLeft' || e.key === 'PageUp') {
      e.preventDefault();
      prevSlide();
    } else if (e.key.toLowerCase() === 's') {
      e.preventDefault();
      toggleNotes();
    } else if (e.key.toLowerCase() === 'f') {
      e.preventDefault();
      toggleFullscreen();
    } else if (e.key === 'Escape' && speakerDrawer?.classList.contains('open')) {
      toggleNotes();
    }
  });

  if (nextBtn) nextBtn.addEventListener('click', nextSlide);
  if (prevBtn) prevBtn.addEventListener('click', prevSlide);
  if (toggleNotesBtn) toggleNotesBtn.addEventListener('click', toggleNotes);
  if (closeNotesBtn) closeNotesBtn.addEventListener('click', toggleNotes);
  if (fullscreenBtn) fullscreenBtn.addEventListener('click', toggleFullscreen);

  // Presentation stopwatch timer
  let secondsElapsed = 0;
  setInterval(() => {
    secondsElapsed++;
    const mins = Math.floor(secondsElapsed / 60);
    const secs = secondsElapsed % 60;
    if (timerText) {
      timerText.textContent = `${mins < 10 ? '0' + mins : mins}:${secs < 10 ? '0' + secs : secs}`;
    }
  }, 1000);

  if (timerBadge) {
    timerBadge.addEventListener('click', () => {
      secondsElapsed = 0;
      if (timerText) timerText.textContent = "00:00";
    });
  }

  // Initialize first slide
  updateSlide(1);
})();
