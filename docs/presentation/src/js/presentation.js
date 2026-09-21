/**
 * ReloBot Presentation Deck Engine
 * Handles slide navigation, keyboard shortcuts, speaker notes drawer, fullscreen, and stopwatch.
 */

(function () {
  let currentSlide = 1;
  const slides = document.querySelectorAll('.slide');
  const totalSlides = slides.length || 7;

  const progressBar = document.getElementById('progressBar');
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

  function updateSlide(index) {
    if (index < 1 || index > totalSlides) return;
    currentSlide = index;

    slides.forEach((slide) => {
      const slideIdx = parseInt(slide.getAttribute('data-index'), 10);
      if (slideIdx === currentSlide) {
        slide.classList.add('active');
      } else {
        slide.classList.remove('active');
      }
    });

    // Progress bar & counter
    const progress = (currentSlide / totalSlides) * 100;
    if (progressBar) progressBar.style.width = `${progress}%`;
    if (slideCounter) {
      slideCounter.textContent = `${currentSlide < 10 ? '0' + currentSlide : currentSlide} / 0${totalSlides}`;
    }

    // Prev/Next buttons state
    if (prevBtn) prevBtn.disabled = currentSlide === 1;
    if (nextBtn) nextBtn.disabled = currentSlide === totalSlides;

    // Update speaker notes
    if (typeof speakerNotes !== 'undefined' && speakerNotesText) {
      speakerNotesText.textContent = speakerNotes[currentSlide] || "";
    }
    if (notesSlideIndicator) {
      notesSlideIndicator.textContent = `[Slide ${currentSlide}/${totalSlides}]`;
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
    if (toggleNotesBtn) {
      toggleNotesBtn.classList.toggle('active', speakerDrawer.classList.contains('open'));
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
