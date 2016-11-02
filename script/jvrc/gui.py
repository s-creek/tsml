#!/opt/tsml/bin/hrpsysjy
import sys
sys.path.append('/home/player/tsml/share/jython')

import os
import time
from java.lang import System
from java.awt import *
from javax.swing import *
from javax.swing.border import BevelBorder 

from guiinfo import *
import math

FILENAME_ROBOTHOST='.robothost'
txt = None 

def reconnect():
  robotHost = txt.getText().strip()
  System.setProperty('NS_OPT', '-ORBInitRef NameService=corbaloc:iiop:'+ robotHost +':2809/NameService')
  try:
    rtm.initCORBA()
    f = open(FILENAME_ROBOTHOST, 'w')
    f.write(robotHost+'\n')
    f.close()
    print ('reconnected')
    return True
  except IOError:
    print ('can not open to write: '+ FILENAME_ROBOTHOST)
  except:
    print ('can not connect to '+ robotHost)
  return False

def setupConsole():
  System.setProperty('NS_OPT', '-ORBInitRef NameService=corbaloc:iiop:localhost:2809/NameService')
  rtm.initCORBA()

def setupRobot():
  setupConsole()
  rncC = rtm.rootnc
  nshC = rtm.nshost
  if reconnect():
    rncR = rtm.rootnc
    nshR = rtm.nshost
    init(nshC, rncC, nshR, rncR)

def restart():
  waitInputConfirm('!! Caution !! \n Push [OK] to restart rtcd ')
  if reconnect():
    ms = rtm.findRTCmanager()
    ms.restart()

def createButton(name, func):
    if not func.__class__.__name__ == 'function':
       return None
    exec('def tmpfunc(e): import time; t1=time.time();'+func.__name__+'(); t2=time.time(); print "['+name+']", t2- t1 ,"[sec]"')
    btn = JButton(label=name, actionPerformed = tmpfunc)
    del tmpfunc
    return btn

def createButtonText(name, func, jtf, defo=None):
  if not func.__class__.__name__ == 'function':
    return None
  if not jtf.__class__.__name__ == 'str':
    return None
  if defo==None:
    exec('global '+jtf+'; '+jtf+' = JTextField(15)')
  else:
    exec('global '+jtf+'; '+jtf+' = JTextField("'+defo+'",15)')
  exec('def tmpfunc(e): tmpio='+jtf+'.getText(); '+func.__name__+'(tmpio)')
  btn = JButton(label=name, actionPerformed = tmpfunc)
  del tmpfunc
  exec('ret = [ btn, '+jtf+' ]')
  return ret

def createFrame():
  global funcList,txt
  #funcList = [["setup rt-system", setupRobot],["restart rtcd", restart], " "] + funcList
  funcList = [["setup rt-system", setupRobot], " "] + funcList
  frm = JFrame("GUI for "+bodyinfo.modelName, defaultCloseOperation = JFrame.EXIT_ON_CLOSE)
  frm.setAlwaysOnTop(True)
  pnl = frm.getContentPane()
  pnl.layout = BoxLayout(pnl, BoxLayout.X_AXIS)
  pnlId = 0
  pnlCount = 0
  MAX_OBJ_NUM = 30
  iomode=0
  for i,func in enumerate(funcList):
    obj = None
    if func.__class__.__name__ == 'str':
      if func == '\n':
        pnlCount = MAX_OBJ_NUM
      else:
        obj = JLabel(func)
    elif func.__class__.__name__ == 'function':
      obj = createButton(func.__name__, func)
    elif func.__class__.__name__ == 'list':
      if len(func) == 2:
        obj = createButton(func[0], func[1])
      elif len(func) == 3:
        #obj, jtf = createButtonText(func[0], func[1], func[2])
        obj, jtf = createButtonText(func[0], func[1], func[1].__name__+'IO')
        iomode = 1
      elif len(func) == 4:
        #obj, jtf = createButtonText(func[0], func[1], func[2], func[3])
        obj, jtf = createButtonText(func[0], func[1], func[1].__name__+'IO', func[3])
        iomode = 1
  
    if obj != None:
      if pnl.getComponentCount() < pnlId+1:
        p = JPanel()
        p.setBorder(BevelBorder(BevelBorder.RAISED))
        p.layout = BoxLayout(p, BoxLayout.Y_AXIS)
        p.setAlignmentY(Component.TOP_ALIGNMENT)
        if pnlId == 0:
          p.add(JLabel("HOSTNAME of ROBOT"))
          if os.path.isfile(FILENAME_ROBOTHOST):
            f = open(FILENAME_ROBOTHOST, "r")
            txt = JTextField(f.readline())
          else:
            txt = JTextField("localhost")
          p.add(txt)
        pnl.add(p)
      pnl.getComponent(pnlId).add(obj)
  
    pnlCount += 1

    if iomode == 1:
      pnl.getComponent(pnlId).add(jtf)
      pnlCount += 1
      iomode = 0

    if pnlCount > MAX_OBJ_NUM:
      print func
      pnlCount = 0 
      pnlId += 1
  return frm
  
if __name__ == '__main__' or __name__ == 'main':
  frm = createFrame()
  frm.pack()
  frm.show()
