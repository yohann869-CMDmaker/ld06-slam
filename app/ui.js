// Habillage de l'interface (maquette Google Stitch « station sol ») : sections de la barre latérale
// repliables, bouton « tout replier », thème sombre / clair. Ne touche pas aux algorithmes : app.js
// garde toute la logique ; ce module ne fait que montrer / cacher et retenir les préférences.
const $ = (id) => document.getElementById(id);
const KEY = 'ld06.folded';

function load() {
  try {
    return new Set(JSON.parse(localStorage.getItem(KEY) || '[]'));
  } catch {
    return new Set();
  }
}

function save(folded) {
  try {
    localStorage.setItem(KEY, JSON.stringify([...folded]));
  } catch {
    // stockage indisponible : le choix vaut pour cette visite
  }
}

const folded = load();
const sections = [...document.querySelectorAll('#sidebar > section')].filter((s) => s.querySelector(':scope > h2'));

for (const sec of sections) {
  const h2 = sec.querySelector(':scope > h2');
  const name = h2.textContent.trim();
  h2.tabIndex = 0;
  h2.setAttribute('role', 'button');
  const apply = () => {
    const f = folded.has(name);
    sec.classList.toggle('folded', f);
    h2.setAttribute('aria-expanded', String(!f));
  };
  const toggle = () => {
    folded.has(name) ? folded.delete(name) : folded.add(name);
    save(folded);
    apply();
    updateFoldButton();
  };
  h2.addEventListener('click', toggle);
  h2.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      toggle();
    }
  });
  apply();
}

function updateFoldButton() {
  $('btnFold').textContent = folded.size >= sections.length ? 'Tout déplier' : 'Tout replier';
}

$('btnFold').addEventListener('click', () => {
  const foldAll = folded.size < sections.length;
  folded.clear();
  for (const sec of sections) {
    const name = sec.querySelector(':scope > h2').textContent.trim();
    if (foldAll) folded.add(name);
    sec.classList.toggle('folded', foldAll);
    sec.querySelector(':scope > h2').setAttribute('aria-expanded', String(!foldAll));
  }
  save(folded);
  updateFoldButton();
});
updateFoldButton();

$('btnTheme').addEventListener('click', () => {
  const next = document.documentElement.dataset.theme === 'light' ? 'dark' : 'light';
  document.documentElement.dataset.theme = next;
  try {
    localStorage.setItem('ld06.theme', next);
  } catch {
    // idem
  }
});
