const tabs = [...document.querySelectorAll('[data-task]')];
const workbench = document.querySelector('.task-workbench');

export function openTask(name) {
  const active = tabs.find((tab) => tab.dataset.task === name);
  if (!active) return;
  window.robotSimulation?.motion?.stop();
  for (const tab of tabs) {
    const selected = tab === active;
    tab.setAttribute('aria-selected', String(selected));
    tab.tabIndex = selected ? 0 : -1;
    document.getElementById(tab.getAttribute('aria-controls')).hidden = !selected;
  }
  workbench.hidden = name === 'operation';
  workbench.scrollTop = 0;
}

for (const [index, tab] of tabs.entries()) {
  tab.addEventListener('click', () => openTask(tab.dataset.task));
  tab.addEventListener('keydown', (event) => {
    const direction = event.key === 'ArrowRight' ? 1 : event.key === 'ArrowLeft' ? -1 : 0;
    if (!direction && !['Home', 'End'].includes(event.key)) return;
    event.preventDefault();
    const next = event.key === 'Home' ? tabs[0] : event.key === 'End' ? tabs.at(-1) : tabs[(index + direction + tabs.length) % tabs.length];
    openTask(next.dataset.task); next.focus();
  });
}
openTask('operation');
