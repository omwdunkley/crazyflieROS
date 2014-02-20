#! /usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

from __future__ import print_function

import roslib; roslib.load_manifest('dynamic_reconfigure')
import sys
import rospy

WXVER = ['2.8', '2.9']
import wxversion
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    sys.stderr.write('This application requires wxPython version %s' % (WXVER))
    sys.exit(1)

import math
import threading
import time
import unicodedata
import wx
import wx.lib.scrolledpanel

import dynamic_reconfigure.client
import rosservice
import sys

row = 0

class MainWindow(wx.Frame):
    def __init__(self, node=None):
        if node is not None:
            title = 'Reconfigure %s' % node
        else:
            title = 'Reconfigure'

        wx.Frame.__init__(self, None, wx.ID_ANY, title, pos=(200, 200), size=(400, 400))

        # Create menu
        self.menubar = wx.MenuBar()
        self.filemenu = wx.Menu()
        self.filemenu.Append(wx.ID_EXIT, 'E&xit', 'Exit the program')
        wx.EVT_MENU(self, wx.ID_EXIT, self.on_exit)
        self.menubar.Append(self.filemenu, '&File')
        self.SetMenuBar(self.menubar)

        # Add reconfigure control
        if node:
            try:
                wx.SetCursor(wx.StockCursor(wx.CURSOR_WAIT))
                wx.Yield()

                self.reconf = dynamic_reconfigure.client.Client(node, timeout=5.0)
            except:
                print("Can't connect to node %s"%node, file=sys.stderr)
                self.Close()
                return
            finally:
                wx.SetCursor(wx.StockCursor(wx.CURSOR_ARROW))
                wx.Yield()
                
            control = DynamicReconfigurePanel(self, self.reconf)

            control.SetMinSize((control.GetSize()[0] + 30, control.GetSize()[1]))
        else:
            control = DynamicReconfigureSelector(self)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(control, 1, wx.EXPAND | wx.ALL, 0)
        self.SetSizer(self.sizer)

        if node:
            self.Fit()
            self.SetMinSize(self.GetSize())

        # Check every 1s if node has shutdown (and close GUI)
        self.timer = wx.Timer(self, wx.ID_ANY)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(1000, False)

    def on_timer(self, event):
        if rospy.is_shutdown():
            self.Close(True)
            self.Refresh()

    def on_exit(self, e):
        self.Close(True)
        self.Refresh()

    def on_error(self):
        self.Raise()

## Dropdown to choose which node to reconfigure
class DynamicReconfigureSelector(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, wx.ID_ANY)

        self.parent       = parent
        self.reconf_panel = None
        self.has_master   = True

        self.combobox = wx.ComboBox(self, wx.ID_ANY, style=wx.CB_READONLY)
        self.update_combobox()
        self.Bind(wx.EVT_COMBOBOX, self.on_combobox)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.combobox, 0, wx.EXPAND)
        self.SetSizer(self.sizer)

        # Refresh node list periodically
        self.timer = wx.Timer(self, wx.ID_ANY)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(5000, False)

    def on_timer(self, event):
        self.update_combobox()

    def update_combobox(self):
        try:
          nodes = dynamic_reconfigure.find_reconfigure_services()
        except rosservice.ROSServiceIOException:
          if self.has_master:
            print("reconfigure_gui cannot connect to master.", file=sys.stderr)
          self.has_master = False
        else:
          if not self.has_master:
            print("reconfigure_gui reconnected to master.", file=sys.stderr)
          self.has_master = True

          self.combobox.SetItems(nodes)
  
          selected_node = self.get_selected_node()
          if selected_node not in nodes:
              self.close_node()               # node has stopped
          elif self.reconf_panel is None:
              self.view_node(selected_node)   # node has restarted

    def get_selected_node(self):
        return str(self.combobox.GetValue())

    def on_combobox(self, event):
        self.view_node(self.get_selected_node())
        
    def view_node(self, node):
        try:
            wx.SetCursor(wx.StockCursor(wx.CURSOR_WAIT))
            wx.Yield()

            try:
                reconf = dynamic_reconfigure.client.Client(node, timeout=5.0)
            except rospy.exceptions.ROSException:
                return
            finally:
                self.close_node()
            
            self.reconf_panel = DynamicReconfigurePanel(self, reconf)

            self.sizer.Add(self.reconf_panel, 1, wx.EXPAND | wx.ALL, 0)
            self.sizer.Layout()

            self.parent.Fit()
            w = self.parent.GetSize()[0]
            h = self.reconf_panel.hbox.GetMinSize()[1] + self.reconf_panel.GetPosition()[1] + (self.parent.GetSize()[1] - self.parent.GetClientSize()[1])
            self.parent.SetMinSize((w, 100))
            self.parent.SetSize((w, h))
            
            if self.reconf_panel is None:
                self.combobox.SetValue('')
            else:
                self.combobox.SetValue(self.reconf_panel.reconf.name)
        finally:
            wx.SetCursor(wx.StockCursor(wx.CURSOR_ARROW))
            wx.Yield()

    def close_node(self):
        if self.reconf_panel is not None:
            self.reconf_panel.close()
            self.reconf_panel.Destroy()
            self.reconf_panel = None

class DynamicReconfigurePanel(wx.lib.scrolledpanel.ScrolledPanel):
    updater = None
    def __init__(self, parent, reconf):
        wx.lib.scrolledpanel.ScrolledPanel.__init__(self, parent, wx.ID_ANY)

        self.parent = parent
        self.reconf = reconf

        config = self.reconf.get_configuration()
        descr  = self.reconf.get_group_descriptions()

        DynamicReconfigurePanel.updater = DynamicReconfigurePanel.BatchUpdater(self.reconf)

        # Create the editors
        self.editors = {}
        self.groups = {}
        
        self._editor_types = {
            'bool':   DynamicReconfigurePanel.BooleanEditor,
            'int':    DynamicReconfigurePanel.IntegerEditor,
            'double': DynamicReconfigurePanel.DoubleEditor,
            'str':    DynamicReconfigurePanel.StringEditor
        }

        self._group_types = {
            '': DynamicReconfigurePanel.PaneGroup,
            'tab': DynamicReconfigurePanel.PaneGroup,
            'collapse': DynamicReconfigurePanel.CollapsibleGroup,
            'hide': DynamicReconfigurePanel.HideableGroup
        }

        self.flags = {
            'bool':   wx.ALIGN_LEFT,
            'int':    wx.EXPAND,
            'str':    wx.EXPAND,
            'double': wx.EXPAND,
        }
        
        self.hbox = wx.BoxSizer(wx.HORIZONTAL)
        
        sizer = wx.GridBagSizer(vgap=1, hgap=4)
        sizer.SetFlexibleDirection(wx.BOTH)
        sizer.AddGrowableCol(1, 0)

        self.add_group_controls(sizer, descr, config)

        self.hbox.Add(sizer, 1, wx.ALL | wx.EXPAND, 4)

        self.SetSizerAndFit(self.hbox)
        self.SetMinSize((self.GetSize()[0] + 30, 10))
        self.SetupScrolling()

        self.reconf.config_callback = self._config_callback

        DynamicReconfigurePanel.updater.start()

    def add_group_controls(self, sizer, group, config):
        sizer.Add(DynamicReconfigurePanel.PaneGroup(self, group, config, top=self), (row, 0), span=(1,2), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL)

    def _config_callback(self, config):
        for name, editor in self.editors.iteritems():
            wx.CallAfter(editor.update_value, config[name])
        for name, group in self.groups.iteritems():
            def find_cfg(grp):
                cfg = None
                for k, v in grp.items():
                    try:
                        if k.lower() == name.lower():
                            cfg = v
                            return cfg
                        else:
                            try:
                                cfg = find_cfg(v)
                                if not cfg == None:
                                    return cfg
                            except Exception as exc:
                                raise exc
                    except AttributeError:
                        pass
                    except Exception as exc:
                        raise exc
                return cfg
            cfg = find_cfg(config)
            try:
                wx.CallAfter(group.update_state, cfg['state'])
            except:
                if not name.lower() == 'default':
                    print("Could not get config for %s"%name, file=sys.stderr)

    def close(self):
        self.updater.stop()
        self.reconf.close()
        
    class BatchUpdater(threading.Thread):
        def __init__(self, reconf):
            threading.Thread.__init__(self)
            self.setDaemon(True)

            self._reconf         = reconf
            self._cv             = threading.Condition()
            self._pending_config = {}
            self._last_pending   = None
            self._stop_flag      = False

        def run(self):
            last_commit = None

            while not self._stop_flag:
                if last_commit >= self._last_pending:
                    with self._cv:
                        self._cv.wait()                 

                last_commit = time.time()
                update = self._pending_config.copy()
                self._pending_config = {}

                try:
                    updated = self._reconf.update_configuration(update)
                except rospy.ServiceException as ex:
                    pass
                except Exception as exc:
                    raise exc

        def update(self, config):
            with self._cv:
                for name, value in config.items():
                    self._pending_config[name] = value

                self._last_pending = time.time()

                self._cv.notify()

        def update_group(self, group):
            with self._cv:
                if not 'groups' in self._pending_config.keys():
                    self._pending_config['groups'] = []
                self._pending_config['groups'].append(group)

                self._cv.notify()

        def stop(self):
            self._stop_flag = True
            with self._cv:
                self._cv.notify()

    class Editor:
        def __init__(self, name):
            self.name = name

        def update_value(self, value):
            pass

        def _update(self, event):
            new_value = self.get_value() 
            if new_value != self.old_value:
                self.update_configuration(new_value)

        def update_configuration(self, value):
            DynamicReconfigurePanel.updater.update({ self.name : value })

        def get_min_max_text(self, min, max):
            return None

    class EnumEditor(Editor, wx.ComboBox):
        def __init__(self, parent, name, val, enum):
            DynamicReconfigurePanel.Editor.__init__(self, name)
            wx.ComboBox.__init__(self, parent, wx.ID_ANY, style=wx.CB_READONLY)
            
            names  = [item['name'] + ' (' + repr(item['value']) + ')' for item in enum]
            values = [item['value'] for item in enum] 
            self.to_value = dict(zip(names, values))
            self.to_name  = dict(zip(values, names))
            
            self.SetItems(names)

            self.update_value(val)
            
            self.Bind(wx.EVT_COMBOBOX, self._update)

        def get_value(self):
            return self.to_value[self.GetValue()]

        def update_value(self, value):
            self.old_value = value
            self.SetValue(self.to_name.get(value, ''))

    class BooleanEditor(Editor, wx.CheckBox):
        def __init__(self, parent, name, value, min, max):
            DynamicReconfigurePanel.Editor.__init__(self, name)
            wx.CheckBox.__init__(self, parent, wx.ID_ANY)
    
            self.update_value(value)
            
            self.Bind(wx.EVT_CHECKBOX, self._update)
                                             
        def get_value(self):
            return self.GetValue()

        def update_value(self, value):
            self.old_value = value
            self.SetValue(value) 
    
    class StringEditor(Editor, wx.TextCtrl):
        def __init__(self, parent, name, value, min, max):
            DynamicReconfigurePanel.Editor.__init__(self, name)
            wx.TextCtrl.__init__(self, parent, wx.ID_ANY, style=wx.TE_PROCESS_ENTER)
    
            self.old_value = value
            
            self.SetValue(value)
            self.Bind(wx.EVT_TEXT_ENTER, self._update)
            self.Bind(wx.EVT_KILL_FOCUS, self._update)
        
        def get_value(self):
            return unicodedata.normalize('NFKD', self.GetValue()).encode('ascii', 'ignore')

        def update_value(self, value):
            self.old_value = value
            self.SetValue(value)
        
    class IntegerEditor(Editor, wx.Panel):
        def __init__(self, parent, name, value, min, max):
            DynamicReconfigurePanel.Editor.__init__(self, name)
            wx.Panel.__init__(self, parent, wx.ID_ANY)

            self.min = min
            self.max = max

            self.delay_slider_updates = False
            self.pending_update_value = None
            
            self.min_label = wx.StaticText(self, wx.ID_ANY, self._text_value(min))
            self.slider    = wx.Slider(self, wx.ID_ANY, minValue=self._slider_value(self.min), maxValue=self._slider_value(self.max), style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            self.max_label = wx.StaticText(self, wx.ID_ANY, self._text_value(max))
            self.text      = wx.TextCtrl(self, wx.ID_ANY, style=wx.TE_PROCESS_ENTER)
            
            self.slider.SetMinSize((60, -1))

            sizer = wx.BoxSizer(wx.HORIZONTAL)
            sizer.Add((2, -1))
            sizer.Add(self.min_label)
            sizer.Add((2, -1))
            sizer.Add(self.slider, wx.EXPAND)
            sizer.Add((2, -1))
            sizer.Add(self.max_label)
            sizer.Add((4, -1))
            sizer.Add(self.text)
            self.SetSizer(sizer)
    
            self.slider.Bind(wx.EVT_LEFT_DOWN, self._slider_left_down)
            self.slider.Bind(wx.EVT_SCROLL, self._slider_update)
            self.slider.Bind(wx.EVT_LEFT_UP, self._slider_left_up)
            self.text.Bind(wx.EVT_TEXT_ENTER, self._text_update)
            self.text.Bind(wx.EVT_KILL_FOCUS, self._text_update)
            
            self.update_value(value)

        def update_value(self, value):
            self.textvalue = self._text_value(value)
            self.text.SetValue(self.textvalue)

            if self.delay_slider_updates:
                self.pending_update_value = value
                return

            self.value = value

            self.slidervalue = self._slider_value(self.value)
            self.slider.SetValue(self.slidervalue)

        def _slider_update(self, event):
            new_slidervalue = self.slider.GetValue()
            if self.slidervalue != new_slidervalue:
                self._change_value(self._inv_slider_value(new_slidervalue))

        def _slider_left_down(self, event):
            self.delay_slider_updates = True
            event.Skip()

        def _slider_left_up(self, event):
            self.delay_slider_updates = False
            if self.pending_update_value is not None:
                self.update_value(self.pending_update_value)
                self.pending_update_value = None
            event.Skip()

        def _text_update(self, event):
            new_textvalue = self.text.GetValue()
            if self.textvalue != new_textvalue:
                self._change_value(self._inv_bound_text_value(new_textvalue))
    
        def _change_value(self, new_value):
            if new_value != self.value:
                self.update_configuration(new_value)

        def _slider_value(self, val):     return val
        def _inv_slider_value(self, val): return val
        def _text_value(self, val):       return str(val)
        def _inv_text_value(self, val):   return int(val)

        def _inv_bound_text_value(self, val):
            try:
                return min(max(self._inv_text_value(val), self.min), self.max)
            except Exception, e:
                return self.value
            
        def get_min_max_text(self, min, max):
            return 'min: %s max: %s' % (self._text_value(min), self._text_value(max))
    
    class DoubleEditor(IntegerEditor):
        def __init__(self, parent, name, value, min, max):
            infinities = [ 1e1000, -1e1000, None ]
            if not min in infinities and not max in infinities:
                self.func  = lambda x: x
                self.ifunc = lambda x: x
            else:
                self.func  = lambda x: math.atan(x)
                self.ifunc = lambda x: math.tan(x)
            self.scale  = (self.func(max) - self.func(min)) / 100
            self.offset = self.func(min)

            DynamicReconfigurePanel.IntegerEditor.__init__(self, parent, name, value, min, max)
        
        def _slider_value(self, val):     return int(round((self.func(val) - self.offset) / self.scale))
        def _inv_slider_value(self, val): return self.ifunc(self.offset + val * self.scale)
        def _text_value(self, val):       return '%.5g' % val
        def _inv_text_value(self, val):   return float(val)

    class GroupView:
        def __init__(self, group, top=None):
            self.group = group
            self.name = group['name']
            self.id = group['id']
            self.type = group['type']
            self.groups = group['groups']
            self.params = group['parameters']
            self.old_state = group['state']
            self.top = top
            self.tab_view = None

        def update_state(self, state):
            pass

        def _update(self, event):
            self.update_configuration(self.get_state())

        def update_configuration(self, state):
            self.group['state'] = state
            DynamicReconfigurePanel.updater.update_group(self.group)

        def add_groups(self, sizer, config, row):
            for g in self.groups:
                if g['type'] == "tab":
                    if self.tab_view is None:
                        self.tab_view = wx.Notebook(self, wx.ID_ANY)
                    self.tab_view.AddPage(self.top._group_types[g['type']](self.tab_view, g, config, top=self.top), g['name'])
                else:
                    sizer.Add(self.top._group_types[g['type']](self, g, config, top=self.top), (row, 0), span=(1,2), flag = wx.EXPAND | wx.ALL)
                    row = row+1
            if self.tab_view is not None:
                sizer.Add(self.tab_view, (row, 0), span=(1,2), flag=wx.EXPAND | wx.ALL)

    class PaneGroup(GroupView, wx.Panel):
        def __init__(self, parent, group, config, top=None):
            DynamicReconfigurePanel.GroupView.__init__(self, group, top) 
            wx.Panel.__init__(self, parent, wx.ID_ANY)
            
            if top is None:
                self.top = self.GetParent()

            sizer = wx.GridBagSizer()
            sizer.AddGrowableCol(1)
            sizer.SetFlexibleDirection(wx.BOTH)
            row = 0
            
            if not self.id == 0:
                if not group['type'] == 'tab':
                    # spacer
                    sizer.Add((10,20),(row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL, span = (1, 2)) 
                    row = row+1
                
                    # Label
                    label_name = group['name'].replace("_", " ")
                    label = wx.StaticText(self, wx.ID_ANY, label_name)
                    sizer.Add(label, (row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL)

                    # Line
                    divider = wx.StaticLine(self, wx.ID_ANY)
                    sizer.Add(divider, (row, 1), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL)
                    row = row+1
            for param_descr in self.params:
                name = param_descr['name']
                type, val, min, max, description = param_descr['type'], config[name], param_descr['min'], param_descr['max'], param_descr['description']
 
                # Label
                label = wx.StaticText(self, wx.ID_ANY, name + ':')
                sizer.Add(label, (row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT)
 
                # Editor
                try:
                    enum = eval(param_descr['edit_method'])['enum']
                    editor = self.top.EnumEditor(self, name, val, enum)
                except:
                    editor = self.top._editor_types[type](self, name, val, min, max)
                self.top.editors[name] = editor
                sizer.Add(editor, (row, 1), flag = self.top.flags[type])
                row = row+1
       
                tooltip_text = description
                min_max_text = editor.get_min_max_text(min, max)
                if min_max_text is not None:
                    tooltip_text += ' ' + min_max_text
                tooltip = wx.ToolTip(tooltip_text)
                tooltip.SetDelay(2000)
                label.SetToolTip(tooltip)

            self.add_groups(sizer, config, row)

            self.SetSizer(sizer)
            self.top.groups[self.name] = self

    class CollapsibleGroup(GroupView, wx.Panel):
        def __init__(self, parent, group, config, top=None):
            DynamicReconfigurePanel.GroupView.__init__(self, group, top)
            wx.Panel.__init__(self, parent, wx.ID_ANY)
            
            sizer = wx.GridBagSizer()
            sizer.AddGrowableCol(1)
            sizer.SetFlexibleDirection(wx.BOTH)
            row = 0

            #self.button = wx.ToggleButton(self, wx.ID_ANY, label = self.name)
            #self.button.SetValue(1)
            #self.Bind(wx.EVT_TOGGLEBUTTON, self.toggle)
            #sizer.Add(self.button, (row,0), span = (1,2))
            #row = row+1

            # spacer
            sizer.Add((10,20),(row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL, span = (1, 2)) 
            row = row+1
            
            # Label
            label_name = group['name'].replace("_", " ")
            label = wx.StaticText(self, wx.ID_ANY, label_name)
            sizer.Add(label, (row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL)

            # Line
            divider = wx.StaticLine(self, wx.ID_ANY)
            sizer.Add(divider, (row, 1), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL)

            self.check = wx.CheckBox(self, wx.ID_ANY)
            self.check.SetValue(self.group['state'])
            self.Bind(wx.EVT_CHECKBOX, self.toggle)
            sizer.Add(self.check, (row, 2))
            row = row+1

            self.sp = wx.Panel(self, wx.ID_ANY)
            ss = wx.GridBagSizer()
            ss.AddGrowableCol(1)
            ss.SetFlexibleDirection(wx.BOTH)
            sr = 0

            for param_descr in self.params:
                name = param_descr['name']
                type, val, min, max, description = param_descr['type'], config[name], param_descr['min'], param_descr['max'], param_descr['description']
 
                # Label
                label = wx.StaticText(self.sp, wx.ID_ANY, name + ':')
                ss.Add(label, (sr, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT)
 
                # Editor
                try:
                    enum = eval(param_descr['edit_method'])['enum']
                    editor = self.EnumEditor(self.sp, name, val, enum)
                except:
                    editor = self.top._editor_types[type](self.sp, name, val, min, max)
                self.top.editors[name] = editor
                ss.Add(editor, (sr, 1), flag = self.top.flags[type])
                sr = sr+1
       
                tooltip_text = description
                min_max_text = editor.get_min_max_text(min, max)
                if min_max_text is not None:
                    tooltip_text += ' ' + min_max_text
                tooltip = wx.ToolTip(tooltip_text)
                tooltip.SetDelay(2000)
                label.SetToolTip(tooltip)

            self.add_groups(ss, config, sr)
            self.sp.SetSizer(ss)
            sizer.Add(self.sp, (row, 0), span=(1,2), flag= wx.EXPAND | wx.ALL)
            self.SetSizer(sizer)

            self.top.groups[self.name] = self

        def toggle(self, event, up=True):
            if up == True:
                self.group['state'] = self.check.GetValue()
                self._update(event)

            old_x, old_y = self.GetTopLevelParent().GetSizeTuple()
            this_x, this_y = self.sp.GetSizeTuple()
            if self.sp.IsShown() and self.get_state() is False:
                for w in self.sp.GetChildren():
                    w.Hide()
                self.sp.Hide()
                self.GetTopLevelParent().Layout()
                y = old_y - this_y
                self.GetTopLevelParent().SetSize((old_x,y))
            elif not self.sp.IsShown() and self.get_state() is True:
                y = old_y + this_y
                self.GetTopLevelParent().SetSize((old_x,y))
                self.sp.Show()
                for w in self.sp.GetChildren():
                    w.Show()
                self.GetTopLevelParent().Layout()
            else:
                self.check.SetValue(self.get_state())

        def get_state(self):
           return self.group['state']

        def update_state(self, state):
           if not state == self.get_state():
              # up=False prevents the configuration from going into an infinite loop
              self.group['state'] = state
              # make sure that the states match
              self.check.SetValue(state)
              self.toggle(None, up=False)
        
        # Override add_groups to use a subpanel
        def add_groups(self, sizer, config, row):
           for g in self.groups:
               sizer.Add(self.top._group_types[g['type']](self.sp, g, config, top=self.top), (row, 0), span=(1,2), flag = wx.EXPAND | wx.ALL)
               row = row+1           

    class HideableGroup(GroupView, wx.Panel):
        def __init__(self, parent, group, config, top=None):
            DynamicReconfigurePanel.GroupView.__init__(self, group, top) 
            wx.Panel.__init__(self, parent, wx.ID_ANY)
            
            if top is None:
                self.top = self.GetParent()

            sizer = wx.GridBagSizer()
            sizer.AddGrowableCol(1)
            sizer.SetFlexibleDirection(wx.BOTH)
            row = 0
            
            if not self.id == 0:
                # spacer
                sizer.Add((10,20),(row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL, span = (1, 2)) 
                row = row+1
                
                # Label
                label_name = group['name'].replace("_", " ")
                label = wx.StaticText(self, wx.ID_ANY, label_name)
                sizer.Add(label, (row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL)

                # Line
                divider = wx.StaticLine(self, wx.ID_ANY)
                sizer.Add(divider, (row, 1), flag = wx.ALIGN_CENTER_VERTICAL | wx.EXPAND | wx.ALL)
                row = row+1
            for param_descr in self.params:
                name = param_descr['name']
                type, val, min, max, description = param_descr['type'], config[name], param_descr['min'], param_descr['max'], param_descr['description']
 
                # Label
                label = wx.StaticText(self, wx.ID_ANY, name + ':')
                sizer.Add(label, (row, 0), flag = wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT)
 
                # Editor
                try:
                    enum = eval(param_descr['edit_method'])['enum']
                    editor = self.top.EnumEditor(self, name, val, enum)
                except:
                    editor = self.top._editor_types[type](self, name, val, min, max)
                self.top.editors[name] = editor
                sizer.Add(editor, (row, 1), flag = self.top.flags[type])
                row = row+1
       
                tooltip_text = description
                min_max_text = editor.get_min_max_text(min, max)
                if min_max_text is not None:
                    tooltip_text += ' ' + min_max_text
                tooltip = wx.ToolTip(tooltip_text)
                tooltip.SetDelay(2000)
                label.SetToolTip(tooltip)

            self.add_groups(sizer, config, row)

            self.SetSizer(sizer)
            self.top.groups[self.name] = self

        def toggle(self):
            old_x, old_y = self.GetTopLevelParent().GetSizeTuple()
            this_x, this_y = self.GetSizeTuple()
            if self.IsShown() and self.get_state() is False:
                self.Hide()
                self.GetTopLevelParent().Layout()
                y = old_y - this_y
                self.GetTopLevelParent().SetSize((old_x,y))
            elif not self.IsShown() and self.get_state() is True:
                y = old_y + this_y
                self.GetTopLevelParent().SetSize((old_x,y))
                self.Show()
                self.GetTopLevelParent().Layout()

        def update_state(self, state):
            if not state == self.get_state():
                self.group['state'] = state
                self.toggle()

        def get_state(self):
            return self.group['state']

if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) > 2:
        print('Usage: reconfigure_gui <node_name>')
        exit(1)

    app = wx.PySimpleApp(clearSigInt=False)
    rospy.init_node('reconfigure_gui', anonymous=True)
    if len(argv) == 2:
        frame = MainWindow(argv[1])
    else:
        frame = MainWindow()
    frame.Show()

    print('reconfigure_gui started')

    try:
        app.MainLoop()
    except KeyboardInterrupt as e:
        pass
    print('exiting')
