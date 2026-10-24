/*!
 * 精简版 CodeMirror 5 dialog addon(本地实现,替代官方 addon/search/dialog.js)
 *
 * 各 CDN 均无法取到官方 dialog.js,而 addon/search/search.min.js 只使用
 * 其 openDialog / openConfirm 两个扩展 API,此处按 5.65 的调用契约实现:
 *   cm.openDialog(template, callback, options)
 *     options: {value, selectValueOnOpen, closeOnEnter, closeOnBlur,
 *               onClose, onKeyDown(event, value), bottom}
 *   cm.openConfirm(template, callbacks)   替换确认条(Yes/No/All/Stop)
 *   CodeMirror.closeDialog(cm)
 */
(function (mod) {
  if (typeof exports === 'object' && typeof module === 'object')
    mod(require('../../lib/codemirror'));
  else if (typeof define === 'function' && define.amd)
    define(['../../lib/codemirror'], mod);
  else
    mod(CodeMirror);
})(function (CodeMirror) {
  'use strict';

  function dialogStack(cm) {
    return cm.state._cmDialogs || (cm.state._cmDialogs = []);
  }

  function removeDialog(cm, node, onClose) {
    if (!node || !node.parentNode) return;
    node.parentNode.removeChild(node);
    var st = cm.state._cmDialogs;
    if (st) {
      var i = st.indexOf(node);
      if (i >= 0) st.splice(i, 1);
    }
    if (typeof onClose === 'function') onClose();
    try { cm.focus(); } catch (e) { /* 编辑器已销毁 */ }
  }

  function appendTemplate(container, template) {
    if (template == null) return;
    if (template.nodeType) container.appendChild(template);
    else container.appendChild(document.createTextNode(String(template)));
  }

  CodeMirror.defineExtension('openDialog', function (template, callback, options) {
    if (!options) options = {};
    var cm = this;
    var closed = false;

    var dlg = document.createElement('div');
    dlg.className = 'CodeMirror-dialog ' +
      (options.bottom ? 'CodeMirror-dialog-bottom' : 'CodeMirror-dialog-top');
    var inner = document.createElement('div');
    appendTemplate(inner, template);
    dlg.appendChild(inner);

    function close() {
      if (closed) return;
      closed = true;
      removeDialog(cm, dlg, options.onClose);
    }
    function submit(ev) {
      var inp = dlg.getElementsByTagName('input')[0];
      callback(inp ? inp.value : '', ev);
    }

    var inputs = dlg.getElementsByTagName('input');
    if (inputs.length) {
      var inp = inputs[0];
      if (options.value != null) inp.value = options.value;
      CodeMirror.on(inp, 'keydown', function (ev) {
        // 优先交给调用方(搜索条借此拦截 findNext/findPrev 等键)
        var keyHandled = false;
        if (typeof options.onKeyDown === 'function') {
          options.onKeyDown(ev, inp.value);
          keyHandled = ev.defaultPrevented;
        }
        if (ev.keyCode === 13) {                 // Enter
          CodeMirror.e_stop(ev);
          // 与官方契约一致:Enter 始终回调(执行搜索/查找下一个);
          // 若调用方 onKeyDown 已自行回调,则不再重复 submit
          if (!keyHandled) submit(ev);
          if (options.closeOnEnter === false) {
            // 持久搜索/替换条:搜索成功后窗口保持不隐藏,
            // 焦点留在输入框(回调内 setSelection 可能把焦点切走,此处夺回)
            inp.focus();
          } else if (options.closeOnEnter !== false) {
            close();
          }
        } else if (ev.keyCode === 27) {          // Esc
          CodeMirror.e_stop(ev);
          close();
        }
      });
      // 持久条(closeOnEnter:false)不响应失焦关闭;
      // 普通条保持失焦自动关闭的官方契约
      if (options.closeOnEnter !== false && options.closeOnBlur !== false) {
        CodeMirror.on(inp, 'blur', function () { setTimeout(close, 120); });
      }
      if (options.selectValueOnOpen) inp.select();
    }

    var btns = dlg.getElementsByTagName('button');
    for (var i = 0; i < btns.length; i++) {
      CodeMirror.on(btns[i], 'click', function (ev) {
        CodeMirror.e_stop(ev);
        submit(ev);
        if (options.closeOnEnter !== false) close();
      });
    }

    var wrapper = cm.getWrapperElement();
    if (options.bottom) wrapper.appendChild(dlg);
    else wrapper.insertBefore(dlg, wrapper.firstChild);
    dialogStack(cm).push(dlg);
    if (inputs.length) inputs[0].focus();
    return dlg;
  });

  CodeMirror.defineExtension('openConfirm', function (template, callbacks) {
    var cm = this;
    var closed = false;
    var dlg = document.createElement('div');
    dlg.className = 'CodeMirror-dialog CodeMirror-dialog-top';
    var inner = document.createElement('div');
    appendTemplate(inner, template);
    dlg.appendChild(inner);

    function close() {
      if (closed) return;
      closed = true;
      removeDialog(cm, dlg, null);
    }

    var btns = dlg.getElementsByTagName('button');
    for (var i = 0; i < btns.length; i++) {
      (function (idx, btn) {
        CodeMirror.on(btn, 'click', function (ev) {
          CodeMirror.e_stop(ev);
          if (idx < callbacks.length) callbacks[idx]();  // Yes / No / All
          else close();                                   // Stop
        });
      })(i, btns[i]);
    }

    var wrapper = cm.getWrapperElement();
    wrapper.insertBefore(dlg, wrapper.firstChild);
    dialogStack(cm).push(dlg);
    return dlg;
  });

  CodeMirror.closeDialog = function (cm) {
    var st = cm.state && cm.state._cmDialogs;
    if (st && st.length) removeDialog(cm, st[st.length - 1], null);
  };
});
