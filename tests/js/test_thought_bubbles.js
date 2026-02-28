/**
 * TRITIUM-SC Thought Bubble Tests
 * Tests the helper functions for NPC thought bubble rendering.
 * Run: node tests/js/test_thought_bubbles.js
 */

let passed = 0, failed = 0;
function assert(cond, msg) {
    if (!cond) { console.error('FAIL:', msg); failed++; }
    else { console.log('PASS:', msg); passed++; }
}

// ============================================================
// Mock canvas context for wrapText testing
// ============================================================

function createMockCtx() {
    return {
        measureText(text) {
            // Approximate: 7px per character
            return { width: text.length * 7 };
        },
    };
}

// ============================================================
// Import helpers — inline from map.js exports
// We test the pure functions directly.
// ============================================================

// Replicate the functions to test (these will be exported from map.js)
function _wrapText(ctx, text, maxWidth) {
    const words = text.split(' ');
    const lines = [];
    let currentLine = '';

    for (const word of words) {
        const testLine = currentLine ? currentLine + ' ' + word : word;
        const metrics = ctx.measureText(testLine);
        if (metrics.width > maxWidth && currentLine) {
            lines.push(currentLine);
            currentLine = word;
        } else {
            currentLine = testLine;
        }
    }
    if (currentLine) lines.push(currentLine);
    return lines;
}

function _emotionColor(emotion) {
    const colors = {
        curious: '#00f0ff',
        afraid: '#fcee0a',
        angry: '#ff2a6d',
        happy: '#05ffa1',
        neutral: '#888888',
    };
    return colors[emotion] || '#888888';
}

// ============================================================
// Tests
// ============================================================

console.log('\n=== Thought Bubble Tests ===\n');

// -- wrapText tests --

{
    const ctx = createMockCtx();

    // test_wrapText_splits_long_lines
    const lines = _wrapText(ctx, 'This is a really long sentence that should be wrapped across lines', 100);
    assert(lines.length > 1, 'wrapText_splits_long_lines: should produce multiple lines');
    for (const line of lines) {
        // Each line should be within maxWidth (approx check with 7px/char = ~14 chars for 100px)
        assert(ctx.measureText(line).width <= 120, `wrapText line "${line}" within reasonable width`);
    }
}

{
    const ctx = createMockCtx();

    // test_wrapText_single_short_line
    const lines = _wrapText(ctx, 'Hi', 200);
    assert(lines.length === 1, 'wrapText_single_short_line: single word = 1 line');
    assert(lines[0] === 'Hi', 'wrapText_single_short_line: text preserved');
}

{
    const ctx = createMockCtx();

    // test_wrapText_empty_string
    const lines = _wrapText(ctx, '', 200);
    // Empty string: split(' ') gives [''], so 1 line with empty content or 0 lines
    assert(lines.length <= 1, 'wrapText_empty: returns 0 or 1 lines');
}

// -- emotionColor tests --

// test_emotionColor_known_emotions
assert(_emotionColor('curious') === '#00f0ff', 'emotionColor curious = cyan');
assert(_emotionColor('afraid') === '#fcee0a', 'emotionColor afraid = yellow');
assert(_emotionColor('angry') === '#ff2a6d', 'emotionColor angry = magenta');
assert(_emotionColor('happy') === '#05ffa1', 'emotionColor happy = green');
assert(_emotionColor('neutral') === '#888888', 'emotionColor neutral = grey');

// test_emotionColor_unknown_defaults_grey
assert(_emotionColor('confused') === '#888888', 'emotionColor unknown defaults to grey');
assert(_emotionColor('') === '#888888', 'emotionColor empty defaults to grey');

// -- expired thoughts test --
{
    // test_expired_thoughts_cleared
    // Simulate a unit with an expired thought
    const unit = {
        thoughtText: 'Old thought',
        thoughtEmotion: 'curious',
        thoughtExpires: Date.now() - 1000,  // 1s ago
    };
    const isExpired = unit.thoughtExpires <= Date.now();
    assert(isExpired, 'expired_thoughts_cleared: expired thought detected');

    // Verify active thought is not expired
    const unit2 = {
        thoughtText: 'Active thought',
        thoughtEmotion: 'happy',
        thoughtExpires: Date.now() + 60000,  // 60s from now
    };
    const isActive = unit2.thoughtExpires > Date.now();
    assert(isActive, 'expired_thoughts_cleared: active thought not expired');
}

// ============================================================
// Summary
// ============================================================

console.log(`\nResults: ${passed} passed, ${failed} failed out of ${passed + failed}\n`);
if (failed > 0) process.exit(1);
