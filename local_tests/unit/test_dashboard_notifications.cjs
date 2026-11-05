// Run with Node 22+: node --test local_tests/unit/test_dashboard_notifications.cjs
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { test } = require('node:test');

class Element {
  constructor(tag = 'div') { this.tagName = tag.toUpperCase(); this.children = []; }
  addEventListener(name, handler) { (this.handlers ||= {})[name] = handler; }
  setAttribute() {}
  append(child) { this.children.push(child); }
  replaceChildren(...children) { this.children = children; }
  contains(child) {
    return this.children.some(node => node === child || (node instanceof Element && node.contains(child)));
  }
}

test('only newly displayed operator requests chime; narration and repeated polls stay quiet', async () => {
  const elements = new Map();
  let now = 0;
  const storage = new Map([['selected', 'session-a']]);
  const context = vm.createContext({
    Node: Element,
    performance: { now: () => now },
    localStorage: { getItem: key => storage.get(key) || null, setItem: (key, value) => storage.set(key, value) },
    document: {
      activeElement: null,
      addEventListener() {},
      getElementById(id) {
        if (!elements.has(id)) elements.set(id, new Element());
        return elements.get(id);
      },
      createElement: tag => new Element(tag),
      querySelectorAll: () => [],
    },
  });
  const source = fs.readFileSync(path.join(__dirname, '../../services/dashboard/static/app.js'), 'utf8');
  // Install production functions/listeners without starting network polling.
  vm.runInContext(source.slice(0, source.lastIndexOf('\nrefresh();')), context);
  assert.equal(vm.runInContext('soundEnabled', context), true, 'sound defaults on');
  let chimes = 0;
  context.beep = () => {
    assert.ok(elements.get('requests').children.length, 'request must be rendered before the chime');
    chimes++;
  };
  const request = id => ({ id, kind: 'help', message: 'Please inspect the scene.', created: 1 });
  const sessions = (requests, id = 'session-a') => ({ sessions: [{ id, task: 'bottle', requests }] });

  context.renderRequests(sessions([]));
  assert.equal(chimes, 0);
  context.renderRequests(sessions([request('first')]));
  assert.equal(chimes, 1);
  context.renderRequests(sessions([request('first')]));
  assert.equal(chimes, 1, 'repeated requests are silent');

  context.api = async () => ({ transcript: [{ kind: 'assistant', time: 'now', text: 'Planning next move.' }] });
  context.renderActivity = () => {};
  await context.refreshActivity();
  await context.refreshActivity();
  assert.equal(chimes, 1, 'transcript narration is silent');

  const input = new Element('textarea');
  elements.get('requests').append(input);
  context.document.activeElement = input;
  context.renderRequests(sessions([request('first'), request('second')]));
  assert.equal(chimes, 1, 'a request deferred while editing is not announced before it appears');
  context.document.activeElement = null;
  context.renderRequests(sessions([request('first'), request('second')]));
  assert.equal(chimes, 2);
  context.renderRequests(sessions([request('second')]));
  context.renderRequests(sessions([]));
  context.renderRequests(sessions([request('second')]));
  assert.equal(chimes, 2, 'removal or reappearance of the same request does not re-alert');
  context.renderRequests(sessions([request('second')], 'session-b'));
  assert.equal(chimes, 3, 'request IDs are scoped to the session');

  now = 59999;
  context.renderRequests(sessions([request('second')], 'session-b'));
  assert.equal(chimes, 3, 'no reminder before 60 seconds');
  const editing = new Element('textarea');
  elements.get('requests').append(editing);
  context.document.activeElement = editing;
  for (let minute = 1; minute <= 10; minute++) {
    now = minute * 60000;
    context.renderRequests(sessions([request('second')], 'session-b'));
    assert.equal(chimes, 3 + minute, 'reminders continue every minute, even while drafting an answer');
    context.renderRequests(sessions([request('second')], 'session-b'));
    assert.equal(chimes, 3 + minute, 'polling does not duplicate a reminder');
  }
  context.document.activeElement = null;
  context.renderRequests(sessions([]));
  now += 60000;
  context.renderRequests(sessions([]));
  assert.equal(chimes, 13, 'reminders stop when the request is answered or removed');

  const exit = { id: 'exit-123', created: 1, message: 'Agent session ended: exited. Return code: 0.' };
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', requests: [], exit_notice: exit }] });
  assert.equal(chimes, 14, 'a silent zero-code exit also chimes');
  now += 60000;
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', requests: [], exit_notice: exit }] });
  assert.equal(chimes, 15, 'unacknowledged exits repeat every minute');
  const findAcknowledge = node => node instanceof Element &&
    (node.children.includes('Acknowledge') ? node : node.children.map(findAcknowledge).find(Boolean));
  findAcknowledge(elements.get('requests')).handlers.click();
  now += 60000;
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', requests: [], exit_notice: exit }] });
  assert.equal(chimes, 15, 'acknowledged exits remain quiet');
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', requests: [],
    exit_notice: { ...exit, id: 'exit-456' } }] });
  assert.equal(chimes, 16, 'a later attempt ending gets a fresh alert');
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', running: true, requests: [],
    idle_notice: { id: 'idle-turn-1', created: 1, title: 'CLAUDE TURN FINISHED', message: 'Waiting for input.' } }] });
  assert.equal(chimes, 17, 'a completed turn chimes even while the CLI is alive');
  now += 60000;
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', running: true, requests: [],
    idle_notice: { id: 'idle-turn-1', created: 1, title: 'CLAUDE TURN FINISHED', message: 'Waiting for input.' } }] });
  assert.equal(chimes, 18, 'idle-turn alerts repeat until acknowledged or work resumes');
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', running: true, requests: [] }] });
  now += 60000;
  context.renderRequests({ sessions: [{ id: 'session-a', task: 'bottle', running: true, requests: [] }] });
  assert.equal(chimes, 18, 'a resumed model turn cancels the idle reminders');

  const muted = vm.createContext({ ...context,
    localStorage: { getItem: key => key === 'dashboardSound' ? 'muted' : null },
  });
  vm.runInContext(source.slice(0, source.lastIndexOf('\nrefresh();')), muted);
  assert.equal(vm.runInContext('soundEnabled', muted), true, 'every launch starts unmuted, including with an old saved mute');
  assert.equal(elements.get('sound-toggle').textContent, 'Sound on · mute');
  assert.equal(elements.get('sound-status').hidden, false, 'browser audio block is shown separately from mute');
});
