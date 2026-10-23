 const btnEn = document.getElementById('btn-en');
  const btnFr = document.getElementById('btn-fr');
  const htmlRoot = document.getElementById('html-root');
  let thom = "helll";

  function setLang(lang){
    document.querySelectorAll('[data-en]').forEach(el => {
      el.innerHTML = el.getAttribute('data-' + lang);
    });
    htmlRoot.setAttribute('lang', lang);
    btnEn.classList.toggle('active', lang === 'en');
    btnFr.classList.toggle('active', lang === 'fr');
  }

  btnEn.addEventListener('click', () => setLang('en'));
  btnFr.addEventListener('click', () => setLang('fr'));
  setLang('en'); // apply English content on page load

  // Scroll-triggered reveal
  const revealEls = document.querySelectorAll('.reveal');
  const io = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting){
        entry.target.classList.add('in');
        io.unobserve(entry.target);
      }
    });
  }, { threshold: 0.12 });
  revealEls.forEach(el => io.observe(el));

  // Nav shadow on scroll
  const navEl = document.querySelector('nav');
  window.addEventListener('scroll', () => {
    navEl.classList.toggle('scrolled', window.scrollY > 12);
  });