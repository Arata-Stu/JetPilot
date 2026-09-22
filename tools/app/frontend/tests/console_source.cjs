const fs = require('node:fs');
const path = require('node:path');
const root = path.join(__dirname, '..');
exports.readAppSource = () => {
  const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
  const parts = [...html.matchAll(/src="\/([^"?]+\.js)"/g)].map(match => match[1]);
  // Feature files use the same classic-script global scope as app.js.
  return parts.filter(name => name === 'app.js' || ['preflight_ui.js', 'list_ui.js', 'training_ui.js', 'transfer_ui.js'].includes(name))
    .map(name => fs.readFileSync(path.join(root, name), 'utf8')).join('\n');
};
