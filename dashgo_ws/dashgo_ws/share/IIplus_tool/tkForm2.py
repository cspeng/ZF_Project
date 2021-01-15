# -*- coding: utf-8 -*-
#tk objects
#from Tkinter import *
from Tkinter import sys, _setit, Tk, Toplevel, Frame, LabelFrame
from Tkinter import StringVar, IntVar, BooleanVar, PhotoImage
from Tkinter import Label, Text, Entry, Menu, OptionMenu, Scrollbar
from Tkinter import Button, Radiobutton, Checkbutton
#tk consts
from Tkinter import UNITS, SUNKEN, ACTIVE, TRUE, FALSE, INSERT, END
from Tkinter import TOP, CENTER, LEFT, RIGHT, BOTH, X, Y, E, WORD
#tk adds-on
from ttkCalendar import dialog_calendar as getDate
import ttk, time, re, atexit
import numpy as np
from pylab import plot, show, ion, figtext
#import ScrolledText # scrolling  abilities

def datetime():
    return time.strftime('%y-%m-%d %H:%M:%S', time.localtime(time.time())) 

#mainFrame
class winApp(Frame):
    """App frame and form V2.0"""
    WIDTH = 960; HEIGHT = 540; DELAY = 100
    MODE= ['None','AUTO PEAK','PEAK','MEASURE']    
    UNIT= ['None','N.m','lbf.in','Kgf.cm']
    HOLD= ["None","1 sec","2 sec","3 sec","4 sec","5 sec"]    
    BOX = ["None","5 Nm","10  Nm","20  Nm","search"] 
    _convert= (1.0, 1.0, 8.850746, 10.197162)    
    
    def __init__(self, DS, rCH):
        """ root transform """
        self.root = Tk()
        self.root.option_add( "*font", "lucida 16" ) #lucida 16
        if (sys.platform=='linux2'):
            self.root.wm_iconbitmap('@favicon.xbm')
        else:
            self.root.wm_iconbitmap('meter.ico')
        self.DS= DS             #locate SQL Data Source
        self.rCH= rCH           #locate redis client
        
        #config Frame variable
        self.reading= None      #reading Box 
        self.filter= None       #filter Box
        self.onCalendar= False  #single instance for Calendar
        self.u= IntVar()        #unit selection
        self.m= IntVar()        #mode selection
        self.ah= IntVar()       #auto hold selection
        self.bx= IntVar()       #box selection
        self.CalMode= BooleanVar()    #calibration Tool
        self.ShapeMode= BooleanVar()    #calibration Tool
        self.uop= None          #user name from tableBox
        self.E2ACK= None        #PEAK transfer state from MB module
        
        #initial win
        self.initFrame()        #build frame on root
        self.readingBox()       #build reading
        self.filterBox()        #build filter
        self.tableBox()         #build datatable
        
        #app setup
        self.onUpdate()        
        self.centerWindow(self.WIDTH, self.HEIGHT)
        self.rCH.set('mbNew',False)
        self.rCH.set('dataSEQ',0)        
        atexit.register(self.winExit)   #note app exit 

    def winExit(self):
        print "\nApp end here."

    def start(self):
        """ start App """
        self.bind_all("<Key>", self.onKeyPressed)
        self.startdate.bind("<Button-1>",self.chooseStart)
        self.enddate.bind("<Button-1>",self.chooseEnd)  
        self.onSetZero()
        #self._job= self.after(self.DELAY, self.onTimeout)
        self._job= self.after(self.DELAY, self.onTimeout)
        self.root.mainloop()

    def onTimeout(self):
        self.dataShow()
        
        #return next loop
        if self.rCH is not None:
            if self.rCH.get('mbNew')!='True':
                self._job= self.after(self.DELAY, self.onTimeout)   

    def onUpdate(self):
        """ mainloop update """
        if self.reading is not None:
            mode= self.MODE[self.m.get()]
            unit= self.UNIT[self.u.get()]
            hold= self.ah.get()
            if self.rCH is not None:
                self.rCH.set('modeAD',mode)            
                self.rCH.set('unitAD',unit)
                self.rCH.set('holdAD',hold)         
            self.reading.config(text=mode)
            self.readunit.config(text=unit)
 
    def onBoxchange(self):
        msg= self.mbBXkey()
        self.m.set(3)
        self.onUpdate()
        if msg!='':
            self.tframe.showBOX(msg)        
        
    def initFrame(self):
        """ frame transform """
        Frame.__init__(self, self.root)
        self.style = ttk.Style()
        self.style.theme_use("default")
        menubar = Menu(self.root)
        self.root.wm_title("PEMS Tools on cloud")
        self.root.attributes("-topmost", True)
        self.root.config(menu=menubar)
        #add menu
        fileMenu = Menu(menubar,tearoff= 0)
        fileMenu.add_command(label="Change User", command= self.onNew)
        fileMenu.add_command(label="log Viewer", command= self.onLog)        
        fileMenu.add_command(label="Exit", command=self.onExit)
        menubar.add_cascade(label="User", menu=fileMenu)
        mode= self.MODE; unit= self.UNIT
        unitMenu = Menu(menubar, tearoff=0)
        modeMenu = Menu(menubar, tearoff=0)
        for i in (1,2,3):
            unitMenu.add_radiobutton(label=unit[i],variable=self.u, value=i, command= self.onUpdate)
            modeMenu.add_radiobutton(label=mode[i],variable=self.m, value=i, command= self.onUpdate)       
        unitMenu.invoke(unitMenu.index(unit[3]))
        modeMenu.invoke(modeMenu.index(mode[3])) 
        menubar.add_cascade(label="Unit", menu=unitMenu)        
        menubar.add_cascade(label="Mode", menu=modeMenu)
        #add bar and other
        self.statusBar() 
        self.otherMenu(menubar)

    def statusBar(self):
        ''' HID bar '''
        Info= "  online: %s Box" % self.BOX[self.bx.get()]
        self.status = Label(self, bd=1, relief=SUNKEN, anchor=E,font=("lucida", 11))
        self.status.config(text=Info,fg='grey')
        self.status.pack(side=TOP, fill=X)

    def otherMenu(self,menubar):
        ''' hold submenu '''
        holdMenu = Menu(menubar, tearoff=0)
        for i in (1,2,3,5):
            holdMenu.add_radiobutton(label=self.HOLD[i],variable=self.ah, value=i, command= self.onUpdate)       
        holdMenu.invoke(holdMenu.index(self.HOLD[2]))   
        menubar.add_cascade(label="PEAK-Hold", menu=holdMenu)
        #box submenu
        boxMenu = Menu(menubar, tearoff=0)  
        for i in (1,2,3,4):
            boxMenu.add_radiobutton(label=self.BOX[i],variable=self.bx, value=i, command= self.onBoxchange)       
        boxMenu.invoke(boxMenu.index(self.BOX[4]))   
        menubar.add_cascade(label="Meter", menu=boxMenu)
        #tool submenu        
        toolMenu = Menu(menubar, tearoff=0)
        toolMenu.add_checkbutton(label="Calbration", onvalue=1, offvalue=False, variable=self.CalMode, command=self.calProc)
        toolMenu.add_checkbutton(label="Shape", onvalue=2, offvalue=False, variable=self.ShapeMode, command= self.shapeProc)
        menubar.add_cascade(label="Tool", menu=toolMenu)

    def unfocus(self):
        ''' HID frame active '''
        self.focus()

    def centerWindow(self,w,h):
        ''' HID position fixed '''
        parent= self.root
        sw = parent.winfo_screenwidth()
        sh = parent.winfo_screenheight()
        x = (sw - w)/2; y = (sh - h)/2-30
        parent.geometry('%dx%d+%d+%d' % (w, h, x, y))
    
    def readingBox(self):
        ''' HID upper interface '''
        self.r= StringVar()
        self.r.set('     ')
        self.sign= StringVar()
        self.sign.set(' ')
        self.reading = LabelFrame(self, padx=8, pady=8, font=("Calibri", 14))
        self.reading.pack(padx=14, pady=10,side=LEFT)

        #main HID
        data= Entry(self.reading,textvariable= self.r,justify=RIGHT,width=6,font=("Calibri",45))
        data['state'] = 'readonly'
        data.pack(side=LEFT)
        self.readunit= Label(self.reading,padx=8,font=("Calibri",22))
        self.readunit.pack(side=LEFT)
        self.readsign= Label(self.reading,textvariable=self.sign,padx=2,font=("Calibri",33))
        self.readsign.pack(side=LEFT)        
        self.abtn = ttk.Button(self.reading, text="﻿置零", width=5, command= self.onSetZero)
        self.abtn.pack(side= RIGHT, padx=10)  

    def filterBox(self):
        ''' HID upper interface '''
        self.filter = LabelFrame(self, text=' - 讀卡掃描 - ', padx=3, pady=6, font=("Calibri", 15))
        self.filter.pack(padx=16, pady=1,side=LEFT) 
        self.ltStart= StringVar()
        self.ltEnd= StringVar()
        lt= time.localtime(time.time())
        lt="%04d-%02d-%02d" % (lt[0:3])
        
        #e.g. self.ltStart.set('2015-09-01')
        self.ltStart.set(lt)
        self.ltEnd.set(lt)
        Label(self.filter,text='起始日期',font=("Calibri", 11)).grid(column=2,row=1)
        Label(self.filter,text='結束日期',font=("Calibri", 11)).grid(column=2,row=2)
        self.startdate= Entry(self.filter,textvariable=self.ltStart,width=10,fg='blue')
        self.startdate.grid(column=3,row=1,padx=6)
        self.enddate= Entry(self.filter,textvariable=self.ltEnd,width=10,fg='blue')
        self.enddate.grid(column=3,row=2, padx=10)       
        self.pack(fill=BOTH, expand=False)

    def tableBox(self):
        ''' HID lower interface '''
        DS= self.DS
        (heading,fetch)= DS.sqlDB()
        tframe= uiFrame(self.root,fetch,DS)
        toolTree(self,tframe,heading) 
        #tframe.user.focus()
        self.tframe= tframe

    #HID actions
    def onSetZero(self):
        self.unfocus()
        if self.rCH is not None:
            time.sleep(0.1)
            self.rCH.set('setzAD',True)
            time.sleep(0.1)

    def onKeyPressed(self, e): 
        #key = e.keysym
        pass

    def chooseStart(self, e):
        if not self.onCalendar:
            self.onCalendar= True
            date= getDate(self.root)
            if date.lt is not None:
                self.ltStart.set(date.lt)
                if self.ltEnd.get() < date.lt:
                    self.ltEnd.set(date.lt)
            self.onCalendar= False
        
    def chooseEnd(self, e):
        if not self.onCalendar:
            self.onCalendar= True
            date= getDate(self.root)
            if date.lt is not None:
                self.ltEnd.set(date.lt)
                if self.ltStart.get() > date.lt:
                    self.ltStart.set(date.lt)
            self.onCalendar= False
    
    #calibration steps
    def calCrit(self,state):
        crit= (self.kref*(1-self.ktol),self.kref*(1+self.ktol)) 
        crittext = 'Verified criteria:\n %1.3f N.m to %1.3f N.m ' % crit
        self.cal['text']= crittext
        self.kstate.set(state)

    def calBox(self):
        self.ktol= 0.005        # 0.5% of tolerance
        self.kref= 1.000        # 5 N.m divide from 1.000 N.m step 1.000 N.m
        self.cal = LabelFrame(self, text='', padx=6,pady=6,font=("Calibri", 14))
        self.abtn.configure(text="next",command= self.onNextCal)
        self.kstate= IntVar()
        self.calCrit(0)
        self.cal.pack(padx=16,pady=10, side=LEFT)
        state= ('below','comply','over')
        txtcol= ('red','green','red')
        for (ix,txt) in enumerate(state):
            kradio= Radiobutton(self.cal, text=txt, value=ix,font=('Helvetica', 14))
            kradio.config(variable=self.kstate,indicatoron=0,selectcolor=txtcol[ix])
            kradio.config(disabledforeground='orange',state='disabled')
            kradio.grid(column=ix+1,row=1,padx=10,pady=10)

#XXX preparing 
    def shapeProc(self):
        if self.ShapeMode.get():
            if self.rCH is not None:
                self.rCH.set('dataShape',[])
                self.rCH.set('mbShape_Req',True)
                while(self.rCH.get('mbShape_Req')=="True"):
                    pass
                y = eval(self.rCH.get('dataShape'))
                Y= np.array(y)
                plot(Y)
                Y= np.abs(Y[Y>0]); argYmax = Y.argmax()
                figtext(0.5,0.56,round(Y.max(),3),size='medium',color='magenta')
                figtext(0.5,0.53,round(Y[argYmax-1],3),size='medium',color='magenta')
                figtext(0.5,0.5,round(Y[argYmax-2],3),size='medium',color='magenta')
                ion(); show()
        self.ShapeMode.set(False)
    
    def calProc(self):
        if self.CalMode.get():
            self.filter.pack_forget()
            self.calBox()
            self.m.set(3)
            self.u.set(1)
            self.onUpdate()
            self.onSetZero()
            self.tframe.withdraw()
            print "Enter Calibration Steps"
        else:
            self.CalMode.set(1)

    def onNextCal(self):
        if self.CalMode:
            self.kref+= 1.0
            if self.kref>5.0:
                self.kref= 1.0
            self.calCrit(0)
            self.m.set(3)
            self.u.set(1)
            self.onUpdate()
            time.sleep(0.2)
            self.unfocus()

    #automated actions
    def mbBXkey(self):
        ''' multiBox handler '''
        if self.rCH.get('mbNew')!='True':
            alt_stat= eval(self.rCH.get('altSTAT')) + ['None']
            mbBX= int(self.rCH.get('mbBX'))
            mbPark= (alt_stat[mbBX]=='Park')
            if(self.bx.get()==4):
                if mbBX>0 and mbPark:
                    self.bx.set(mbBX)
            okREQ= (mbBX==self.bx.get())
            infoKey= 'online' if (okREQ and mbPark) else 'PLUGIN'
            infoKey= 'REQUIRE' if (not okREQ and mbPark) else infoKey
            infoCtn= self.rCH.get('mbCtn')
            infoCol= {'REQUIRE':'orange','online':'grey','PLUGIN':'orange'}
            mbBX= self.BOX[self.bx.get()]
            mbBX= '' if mbBX=='search' else mbBX
            Info= '%s %s Meter' % (infoKey, mbBX) 
            sInfo= ['','one','two','three']
            sInfo= sInfo[alt_stat[1:4].count('None')]
            sInfo= ',  %s box not connected yet!   ' % sInfo if sInfo!='' else '   '
            sInfo= Info + sInfo + str(infoCtn)
            self.status.config(text=sInfo, fg=infoCol[infoKey])
            Info= '' if infoKey=='online' or mbBX=='' else Info
        else:
            Info= ''
        #sync RQ BOX on STM32 and zero if in search mode
        self.rCH.set('mbBXRQ',self.bx.get() & 0x03)    
        return Info

    def transferE2(self):
        ''' trigger peak-log e2 '''
        E2BUSY= self.rCH.get('E2BUSY')

        #transfer input E2BUSY and output E2e2
        # sequence: E2BUSY, !E2e2, E2ACK, !E2BUSY, E2e2, !E2ACK
        if E2BUSY=='1':
            self.rCH.set('E2e2', 0)
        if self.E2ACK=='1' and E2BUSY=='0':
            self.rCH.set('E2e2', 1)
        self.E2ACK= E2BUSY
        
    def isCal(self, data):
        ''' do calibration mode as if '''
        if self.CalMode:
            try:
                reading= float(data)
                crit= (self.kref*(1-self.ktol),self.kref*(1+self.ktol)) 
                kstate= 0 if reading < crit[0] else 1
                kstate= 2 if reading > crit[1] else kstate
                self.calCrit(kstate)
            except:
                pass        

    def dataShow(self):
        ''' HID automation '''
        if self.rCH is not None:
            self.mbBXkey()
            #measurment data and sign            
            data= self.rCH.get('dataAD')
            sign= ('↶','◌','↻')[int(self.rCH.get('signAD'))]
            self.r.set(data)
            self.sign.set(sign)
            self.update()
            self.transferE2()
            self.isCal(data)
            seq = int(self.rCH.get('dataSEQ'))
            self.rCH.set('dataSEQ', seq+1)

    #HID states
    def cancel(self):
        if self._job is not None:
            self.after_cancel(self._job)
            self._job = None

    def onNew(self):
        self.rCH.set('mbNew',True)
        self.cancel()
        self.root.destroy()
        print "\n--- Change New User on %s ---" % datetime()

    def onLog(self):
        #append stdlog
        self.DS.stdlogClose()
        self.DS.stdlogOpen()        
        tkViewer(self.root,{'winfo_xy':self.tframe.winfo_xy()})		
		
    def onExit(self):
        self.rCH= None
        self.cancel()
        self.root.destroy()
        print "App quit..."        


class tkdialog(Toplevel):
    """interface dialog"""
    def __init__(self, parent, fetch, rCH = None, tool = None, remark=None):
        Toplevel.__init__(self, parent)
        self.option_add( "*font", "lucida 14" ) 
        self.transient(parent)
        self.parent = parent
        self.fetch= fetch
        self.rCH= rCH
        self.tool= tool
        self.remark= remark
        self.result = None
        body = Frame(self)
        self.initial_focus = self.body(body)
        body.pack(padx=5, pady=5)
        self.buttonbox()
        # set initial_focus on this Toplevel
        if not self.initial_focus:
            self.initial_focus = self
        if fetch.has_key('winfo_xy'):
            (x,y)= fetch['winfo_xy']
        else:
            x= parent.winfo_rootx() + 320
            y= parent.winfo_rooty() - 150
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.geometry('%dx%d+%d+%d' % (450, 325, x, y))
        self.initial_focus.focus_set()
        self.wait_window(self)

    def body(self, master):
        # overridden method of dialog body
        pass

    def buttonbox(self):
        # add standard button box.
        box = Frame(self)
        self.ret= Button(box, text='OK', width=10, command=self.ok, default=ACTIVE)
        self.ret.pack(side=LEFT, padx=5, pady=5)
        ebtn = Button(box, text="Cancel", width=10, command=self.cancel)
        ebtn.pack(side=LEFT, padx=5, pady=5)
        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.cancel)
        box.pack()

    # standard button semantics
    def ok(self, event=None):
        if not self.validate():
            self.initial_focus.focus_set() # put focus back
            self.cancel()
            return
        self.withdraw()
        self.update_idletasks()
        self.apply()
        self.cancel()

    def cancel(self, event=None):
        # put focus back to the parent window
        self.parent.focus_set()
        self.destroy()

    # command hooks
    def validate(self):
        return 1 # override

    def median(self,x):
        if len(x)%2 != 0:
            return sorted(x)[len(x)/2]
        else:
            midavg = (sorted(x)[len(x)/2] + sorted(x)[len(x)/2-1])/2.0
            return midavg

    def apply(self):
        pass # override
		

class tkViewer(tkdialog):
    """ App Log Viewer """
    def body(self, master):
        self.title('Log Viewer')
        scrollbar = Scrollbar(master)
        scrollbar.pack(side=RIGHT, fill=Y)  
       
        text = Text(master, wrap=WORD, yscrollcommand=scrollbar.set)
        fname= 'log\stdlog.txt'
        flog= open(fname, "r")
        stream = flog.read()
        text.insert(END, stream)
        text.pack(fill=BOTH, expand=1)
        scrollbar.config(command=text.yview) 

        self.after(16000, self.cancel)


#XXX table Tree     
class toolTree(object):
    """treeview with jobno icons"""
    tcwidth= (100,100,100,100) + (100,100,125) + (100,0,75)
    tcdisplay= (8,3,4,6,7,9)

    def __init__(self, app, ui,columns):
        master= ui.bi       # background image from ui_widgets               
        tree = ttk.Treeview(master)
        tree["columns"]= tuple(columns)         #column header
        tree.configure(displaycolumns=self.tcdisplay)
        tree.configure(height=50,padding=(5,2)) #50
        for (i,col) in enumerate(columns):
            tree.column(col, width= self.tcwidth[i])
            tree.heading(col, text=col, command=lambda c=col: self.sortby(c, 0))
        tree.column('#0', width=320)
        tree.heading('#0', text='%s/%s' % tuple(columns[1:3]))
        tree.heading('#0', command=lambda c=columns[2]: self.sortby(c, 0))
        self.hsb = ttk.Scrollbar(orient="horizontal", command=tree.xview)
        self.vsb = ttk.Scrollbar(orient="vertical", command=tree.yview)
        self.tree= tree
        self.ui= ui                     #object app.tframe of class uiFrame
        ui.extend(app,self)             #refer fn app.tframe.extend
        
    def start(self):
        self.ui.start()

    def grid(self,master):
        tree= self.tree
        tree.configure(yscrollcommand=self.vsb.set)
        tree.configure(xscrollcommand=self.hsb.set)
        #tree.bind("<Double-1>", self.onRowClick)        
        tree.bind("<<TreeviewSelect>>", self.onRowSelect)
        tree.bind("<B1-Motion>",lambda e, s=self: s.onB1motion(e.x,e.y))
        #self.hsb.grid(sticky='ew', in_= master)
        self.vsb.grid(column=1, row=0, sticky='ns', in_= master)
        tree.grid(column=0, row=0, sticky='nsew', in_ = master)

    def withdraw(self):
        self.hsb.destroy()
        self.vsb.destroy()
        self.tree.destroy()

    def getIID(self,jn=''):
        tree= self.tree
        if jn=='':
            child= sum([tree.get_children(p) for p in tree.get_children('')],())
        else:
            child= tree.get_children(jn)
        return child

    def sortby(self, acol, descending):
        """sort tree contents when a column header is clicked on"""
        tree= self.tree
        for jn in tree.get_children(''):
            items = [(tree.set(c, acol), c) for c in self.getIID(jn)]
            items.sort(reverse=descending)
            for (ix, item) in enumerate(items):
                tree.move(item[1], jn, ix)
        # switch the heading so it will sort in the opposite direction
        tree.heading(acol, command=lambda c=acol: self.sortby(c, int(not descending)))
        key= self.ui.ui['key'].get()
        aix= self.tree['columns'].index(acol)
        OPTIONS= [val[aix] for val in self.getHints(key)[1]]
        self.alt_options(acol,OPTIONS)

    def onB1motion(self,x,y):
        sign = lambda a: (bool(a > 0) - bool(a < 0))
        ui= self.ui.ui; tree= self.tree
        b0xy= [x,y,0,0] if ui['b0xy'] is None else ui['b0xy']
        b0xy[2]+= b0xy[0] - x
        b0xy[3]+= b0xy[1] - y
        if abs(b0xy[3]) > 16:
            sdir= sign(b0xy[3])
            b0xy[3]-= sdir*16
            tree.yview_scroll(sdir,UNITS)
            if len(tree.selection()):
                selitem= tree.selection()[0]
                if tree.item(selitem,"values")=='':
                    tree.item(selitem, open=FALSE)                
        ui['b0xy']= [x,y] + b0xy[2:4]                        
        return 'break'

    def onUpdate(self):
        key= self.ui.ui['key'].get(); tree= self.tree
        sels= tree.selection()
        if len(sels)!=0:
            fetch= tree.item(sels[0] ,"values")
            if key in tree.get_children('') and len(fetch)==0:
                if tree.item(key,'open'):
                    tree.item(key, open=FALSE)
                else:
                    tree.item(key, open=TRUE)

    def onRowClick(self, event):
        self.ui.onRowClick()

    def onRowSelect(self, event):
        ui= self.ui.ui; ui['b0xy']= None
        self.ui.NUMhide()
        acol= ui['acol'].get()
        aix= self.tree["columns"].index(acol)         
        item = self.tree.selection()[0]
        fetch= self.tree.item(item,"values")
        kcol= ui['kcol'];  kix= ui['kix']   #key JobNo; default column 3
        key= fetch[kix] if len(fetch)!=0 else item
        alt= fetch[aix] if len(fetch)!=0 else ''
        self.update_selection(kcol,key,acol,alt)

    def update_selection(self,kcol,key,acol,alt):
        altdata= []; tree= self.tree
        aix= tree['columns'].index(acol) 
        altdata= [self.getValues(item)[aix] for item in self.getIID(key)]
        if len(altdata):
            self.alt_options(acol,altdata)
        if len(altdata)!=1:
            self.ui.ui['fbtn'].config(text= "")
        #self.ui.ui['key'].set(key)
        self.ui.ui['alt'].set(alt)
        self.onUpdate()

    def alt_options(self,acol,altdata):
        ui= self.ui.ui
        ui['acol'].set(acol)
        alt= ui['alt']
        opts= ui['opts']
        opts['menu'].delete(0, 'end')
        alt.set('')
        new_OPTIONS= list(sorted(set(altdata)))
        if len(new_OPTIONS):
            for choice in new_OPTIONS:
                opts['menu'].add_command(label=choice, command=_setit(alt, choice))

    def getHints(self, jn):
        jnhints= []; values= []; tree= self.tree
        jnlist= tree.get_children('')
        jnlist= list(sorted(set(jnlist)))
        jnlen= len(jn)
        for item in jnlist:
            if jn==str(item)[:jnlen]:
                jnhints.append(item)
                for iid in tree.get_children(item):
                    values.append(tree.item(iid)['values'])
        if len(jnhints)==0:
            values= self.getValues()
        return (jnhints, values)

    def delValues(self):
        # grab values to delete
        for jn in self.tree.get_children(''):
            self.tree.delete(jn) 

    def addValues(self,fetch):
        #set-up icons by JobNo and insert option: open=TRUE
        try:
            if fetch is not None:
                jicon= [s[1] for s in fetch]
                jicon= list(sorted(set(jicon)))
                wait= self.ui.ui['wait']
                for jn in jicon:
                    jwait= [item[1]==jn and item[9]==wait for item in fetch]
                    jtext= '%s  {%d wait}' % (jn, sum(jwait))
                    self.tree.insert("",1,jn,text=jtext)
                for item in fetch:
                    item= list(item)
                    item[7]+= "%s" % item[8]; item[8]= ''
                    jn= item[1]; iid= item[0]
                    #print(jn,"end",iid, item[2], item) when debug
                    self.tree.insert(jn,"end",iid, text=item[2], values=item)
            acol= self.ui.ui['acol'].get()
            #aix= self.tree['columns'].index(acol)
           # OPTIONS= [val[aix] for val in self.getValues()]
            OPTIONS= []
            self.alt_options(acol,OPTIONS)
        except:
            print("database fetching error")

    def getValues(self,iid=''):
        values= []; tree= self.tree
        if iid=='':
            for c in self.getIID():
                values.append(tree.item(c)['values'])
        else:
            values= tree.item(iid)['values']
        return values


class _PopupNumKey(Toplevel):
    '''A Toplevel instance that displays a keyboard'''
    def __init__(self, parent, attach, x, y, keycolor, keysize=5):
        Toplevel.__init__(self, takefocus=0)

        self.overrideredirect(True)
        self.attributes("-topmost", True)
        self.attributes('-alpha',0.85)
        self.option_add( "*font", "lucida 32" ) 
        self.parent = parent
        self.attach = attach
        self.parent.onNumKey= self
        self.keysize = keysize
        self.keycolor = keycolor
        self.x = x if x >150 else 150
        self.y = y

        self.row1 = Frame(self)
        self.row2 = Frame(self)
        self.row3 = Frame(self)
        self.row4 = Frame(self)

        self.row1.grid(row=1)
        self.row2.grid(row=2)
        self.row3.grid(row=3)
        self.row4.grid(row=4)
        
        self._init_keys()

        # destroy _PopupKeyboard on keyboard interrupt
        self.bind('<Key>', lambda e: self.hide())
        self.bind('<FocusOut>', lambda e: self.hide())

        # resize to fit keys
        self.update_idletasks()
        self.geometry('{}x{}+{}+{}'.format(self.winfo_width(),
                                           self.winfo_height(),
                                           self.x,self.y))
        self.focus_set()
        self.wait_window(self)        

    def _init_keys(self):
        self.alpha = {
            'row1' : ['1','2','3','[ BS ]'],
            'row2' : ['4', '5', '6','>>>'],
            'row3' : ['7','8','9','0']
            }
        
        for row in self.alpha.iterkeys(): # iterate over dictionary of rows
            if row == 'row1':             # TO-DO: re-write this method
                i = 1                     # for readability and functionality
                for k in self.alpha[row]:
                    Button(self.row1,
                           text=k,
                           width=self.keysize,
                           bg=self.keycolor,
                           command=lambda k=k: self._attach_key_press(k)).grid(row=0,column=i)
                    i += 1
            elif row == 'row2':
                i = 2
                for k in self.alpha[row]:
                    Button(self.row2,
                           text=k,
                           width=self.keysize,
                           bg=self.keycolor,
                           command=lambda k=k: self._attach_key_press(k)).grid(row=0,column=i)
                    i += 1
            else:
                i = 2
                for k in self.alpha[row]:
                    Button(self.row3,
                           text=k,
                           width=self.keysize,
                           bg=self.keycolor,
                           command=lambda k=k: self._attach_key_press(k)).grid(row=0,column=i)
                    i += 1

    def hide(self,e=None):
        self.withdraw()
    
    def show(self):
        """"""
        self.update()
        self.deiconify()
       
    def _destroy_popup(self):
        self.destroy()

    def _attach_key_press(self, k):
        if k == '>>>':
            self.attach.focus_set()
            self.parent.isValid()
        elif k == '[ BS ]':
            icur= self.attach.index(INSERT)
            self.attach.delete(icur-1, icur)
        elif k == '[ space ]':
            self.attach.insert(END, ' ')
        else:
            icur= self.attach.index(INSERT)
            self.attach.insert(icur, k)


def PopupNumKey(parent, attach, keysize=4, keycolor='gray'):
    """ Numerical keyboard Popup """
    kb = _PopupNumKey(parent= parent,
                         attach= attach,
                         x=attach.winfo_rootx() - 375,
                         y=attach.winfo_rooty() + 50,
                         keysize= keysize,
                         keycolor= keycolor)
    return kb


class winMessage(tkdialog):
    """ App Message Box """
    #fetch={'winfo_xy':(x,y), 'title':title,
    #'message':message, 'fgcolor':color}
    def body(self, master):
        self.title(self.fetch['title'])
        fgcolor= self.fetch['fgcolor']
        font= ("Calibri", 52) if self.fetch['title']=='Result' else ("Calibri", 15)
        Label(master, text=self.fetch['message'], fg=fgcolor, font=font).grid(sticky='w',pady=80)
        self.after(1200, self.cancel)

    def buttonbox(self):
        # add standard button box.
        box = Frame(self)
        self.ret= Button(box, text='OK', width=10, command=self.ok, default=ACTIVE)
        self.ret.pack(side=LEFT, padx=5, pady=5)
        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.ok)
        box.pack()


class winDialog(tkdialog):
    """ App selection entry """         
    def body(self, master):
        self.title('Torque Interface')
        Pinfo= tuple(self.fetch['values'][2:5])
        Prange= " %s - %s  " % (self.fetch['spec'][0],self.fetch['spec'][1])
        Punit= self.fetch['spec'][2]
        Label(master, text="").grid(row=0, sticky='w')
        Label(master, text="").grid(row=2, sticky='w')          
        Label(master, text="Model:").grid(row=3, sticky='w')  
        Label(master, text=Pinfo[0]).grid(row=3,column=1,sticky='w')
        Label(master, text="Process-").grid(row=4, sticky='w') 
        Label(master, text=" Code: %s  /  Name: %s" % Pinfo[1:3]).grid(row=4,column=1,sticky='w')        
        Label(master, text="Range:").grid(row=5, sticky='w')        
        Label(master, text=Prange + Punit).grid(row=5,column=1,sticky='w')
        Label(master, text="Operator:").grid(row=6, sticky='w')
        Label(master, text=self.fetch['values'][5]).grid(row=6,column=1,sticky='w')
        Label(master, text="",font=("Calibri",1)).grid(row=7, sticky='w')  
        Label(master, text="",font=("Calibri",2)).grid(row=9, sticky='w')
        Label(master, text="scan Tool...").grid(row=1, sticky='w')

        self.e1 = Entry(master)
        self.e1.grid(row=1, column=1)
        if self.tool is not None:
            self.e1.insert(0, str(self.tool))
            self.e1.config(state='readonly')

        self.cb = Checkbutton(master, text="Forward rotation",state='disabled')
        self.cb.select()
        self.cb.grid(row=8, columnspan=2, sticky='w')
        self.result= (None, None)
        self.e1.focus_force()

    def apply(self):
        self.result= (self.fetch, self.tool)

    # ret button semantics
    def ok(self, event=None):
        if not self.isValid():
            self.e1.delete(0, END)
            self.e1.focus_force()            
            return
        self.withdraw()
        self.update_idletasks()
        self.apply()
        self.cancel()

    def isValid(self,event=None):
        if self.tool is None:
            tid= self.e1.get()
            if tid is not None:
                tid=tid.strip()
                try:
                    if len(tid)==10:                
                        tid= re.search(r'^00(\d{8})',tid).group(1)
                        self.tool= tid
                        return True

                except:
                    pass

            return False  

        else:
            return True
        

class toolDialog(tkdialog):
    """App Tool Measurement"""
    def body(self, master):
        self.DELAY= 50
        self.title('Torque Measurement')
        self.range= "%s - %s  " % (self.fetch['spec'][0],self.fetch['spec'][1])
        self.unit= self.fetch['spec'][2]
        Label(master, text="Tool:").grid(row=0, sticky='w')
        Label(master, text="Range:").grid(row=1, sticky='w')
        Label(master, text="Operator:").grid(row=2, sticky='w')
        Label(master, text="",font=("Calibri",1)).grid(row=3, sticky='w')
        Label(master, text="Readings:").grid(row=4, sticky='w')        
        Label(master, text=self.tool).grid(row=0,column=1)
        Label(master, text=self.range+self.unit).grid(row=1,column=1)
        Label(master, text=self.fetch['values'][5]).grid(row=2,column=1)
        Label(master, text="",font=("Calibri",1)).grid(row=5, sticky='w')
        Label(master, text="Remark:").grid(row=6,sticky='w')        
        self.e2 = Text(master,height=5,width=20,padx=6,pady=3,spacing1=3)
        self.e2.grid(row=4, column=1, columnspan=2,sticky='ew',padx=6)
        #remark Box
        self.combo= ttk.Combobox(master,values=self.remark,state='readonly')
        self.combo.grid(row=6,column=1,columnspan=2,pady=3,padx=5,sticky='w')
        self.combo.current(0)
        self.combo['state']= 'disabled'
        self.init_values()        
        if self.rCH is not None:
            self.rCH.set('E2e2',0)        
            self.after(self.DELAY, self.afterTimeout)   # loop every 50ms

    def init_values(self):
        self.step= 0
        self.retctn= 20*1.2
        self.values= [None,None,None,None,None,None]
        self.result= (None, None, None,None,None)        
        self.e2.delete('1.0', END)

    def apply(self):
        if self.step >= 3:
            self.values[3]= round(self.median(self.values[0:3]),3)
        ret= self.ret.cget('text')
        self.result= (self.fetch,self.tool,self.values,ret,self.combo.get())

    # ret button semantics
    def ok(self, event=None):
        retSeq= self.ret.cget('text')
        if retSeq in ('START','RETEST'):
            self.init_values()
            return
        #process after timeout
        if self.step==3:
            retSeq= 'PASS' if self.validate() else 'FAIL'
            self.ret.config(text=retSeq)
        if retSeq== 'FAIL':
            self.combo['state']= 'enabled'
            self.combo.focus_force()
            if self.combo.get()=='':
                print 'overall test failed'
                return
        if retSeq== 'PASS' and self.retctn>0:
            return
        self.withdraw()
        self.update_idletasks()
        self.apply()
        self.cancel()
 
    def validate(self):
        flag= (self.step==3)
        if flag:
            for E2PEAK in self.values[0:3]:
                E2pass= False if  E2PEAK < self.fetch['spec'][0] else True
                E2pass= False if  E2PEAK > self.fetch['spec'][1] else E2pass
                flag= flag and E2pass
        return flag               
        
    def afterTimeout(self):
        E2e2= self.rCH.get('E2e2')
        
        # peak log if E2e2
        if E2e2 == '1' and self.step < 3:       #log count
            self.rCH.set('E2e2',0)
            E2PEAK= float(self.rCH.get('E2PEAK'))
            E2PEAK= round(E2PEAK, 2) if self.unit != 'N.m' else E2PEAK
            self.values[self.step]= E2PEAK
            self.step+= 1
            E2FLAG= "FAIL" if  E2PEAK < self.fetch['spec'][0] else "PASS"
            E2FLAG= "FAIL" if  E2PEAK > self.fetch['spec'][1] else E2FLAG
            E2text= "  %1.3f        %s\n" % (E2PEAK,E2FLAG)
            self.e2.insert(END,E2text)
            #self.e2.see(END)

        # user response sequence
        # START, RETEST, validate, PASS, FAIL
        retSeq= self.ret.cget('text')
        if self.step >= 3:
            if retSeq=='RETEST':
                retSeq= 'validate'
                self.ret.config(text=retSeq)
                self.ok()
        else:
            retSeq= 'START' if self.step == 0 else 'RETEST'
            self.ret.config(text=retSeq)
        if retSeq=='PASS':
            self.retctn-= 1
            if self.retctn <= 0:
                self.ok()
                return
                
        self.after(self.DELAY, self.afterTimeout)           


class ui_widgets(object):
    """ UI interface prototype """     
    def __init__(self,root,fetch,DS=None):
        #init user field
        #historical 'kcol':'JobNo'
        ui= {'key':StringVar(), 'kcol':'掃描 ID', 'kix': 1,
             'alt':StringVar(), 'acol':StringVar(), 'aix': 4,
             'ulab':StringVar(), 'user':None, 'opts':None, 
             'uid':None, 'wait':None, 'allbtn':None,'fbtn':None,'dfid':None}
        self.ui_style(root)
        #frame and background
        fh = ttk.Frame()
        fh.grid(column=0, row=0, sticky='nwes')
        fh.grid_columnconfigure(0, weight=1)
        fh.grid_rowconfigure(0, weight=1)
        bi= self.bg_logo()
        fh.pack(fill='both', expand=True)  
        bi.pack(fill='both', expand=True) 
        #user interface
        allbtn= Button(text= "全部更新",font=("Calibri", 14), command= self.onAll)
        fbtn= Button(text= "",font=("Calibri", 14))     #, command= self.onTool
        user= Entry(textvariable=ui['key'])
        user.config(fg='blue', width='10', font=('lucida', '20'))
        if fetch is not None:
            opts_alt= tuple(sorted(set([vals[3] for vals in fetch])))
        else:
            opts_alt= ('',)
        opts= apply(OptionMenu, (fh, ui['alt']) + opts_alt)
        opts.config(fg='blue', width='23', font=('lucida', '16'))
        allbtn.bind('<Enter>', lambda e: self.NUMhide() )
        fbtn.bind('<Enter>', lambda e: self.NUMhide() )
        opts.bind('<Enter>', lambda e: self.NUMhide() )
        Label(textvariable= ui['ulab'],font=("Calibri", 11)).grid(row=1, column=2, in_ = fh) 
        if DS.isConn():
            ui['ulab'].set('User')
            user.grid(row=1, column=3, padx=8, in_ = fh)
        else:
            ui['ulab'].set('not connect  ')
        (ui['allbtn'], ui['fbtn'])= (allbtn, fbtn)
        (ui['user'], ui['opts'], ui['wait'])= (user, opts, DS.Electric_state[1])
        #explode public
        self.tt= None
        self.onNumKey= None
        (self.DS, self.root, self.fetch)= (DS, root,  fetch)
        (self.fh, self.bi, self.ui)= (fh, bi, ui)


    def ui_style(self,root):
        #root.option_add( "*font", "lucida 15" ) 
        style = ttk.Style()
        style.theme_use("default")
        style.configure('.', font=('lucida', 15))


    def extend(self):
        print "not implement yet"

    def onAll(self):
        print "not implement yet"

    def onFilter(self):
        print "not implement yet"

    def onTool(self):
        print "not implement yet"

    def onRowClick(self):
        print "not implement yet"
        
    def NUMhide(self):
        print "not implement yet"        

    def withdraw(self):
        self.tt.withdraw()
        self.fh.forget()

    def bg_logo(self):
        #logo image
        logo= PhotoImage(file="logo.gif")
        bi= Label(compound= CENTER, image=logo)
        bi.image= logo          
        bi.grid(column=0, row=0, sticky='nwes')
        bi.grid_columnconfigure(0, weight=1)
        bi.grid_rowconfigure(0, weight=1)
        return bi

    def start(self):
        self.root.mainloop()


class uiFrame(ui_widgets):
    """ content view """        
    def extend(self, app, tt):
        self.app= app
        self.tt= tt
        ui= self.ui
        ui['b0xy']= None
        ui['acol'].set(tt.tree["columns"][ui['aix']])
        tt.addValues(self.fetch)        
        self.isRowDialog= False
        self.user= self.ui['user']
        #self.user.bind("<Button-1>",self.NumInput)
        self.user.bind('<Return>', self.isValid)

        self.user.focus_force()  

    def onAll(self):
        tt= self.tt
        uid= self.ui['uid']
        starttime= self.app.ltStart.get()
        endtime= self.app.ltEnd.get()          
        df= self.DS.sqlExpr(uid,starttime,endtime)
        aix= self.ui['aix']
        self.ui['acol'].set(tt.tree["columns"][aix])
        self.fetch= df
        tt.delValues()
        tt.addValues(df)
        self.ui['key'].set('')
        self.ui['alt'].set('')
        self.ui['fbtn'].config(text= "")

    def onFilter(self):
        tt= self.tt; ui= self.ui
        uid= ui['uid']
        if uid is not None:
            starttime= self.app.ltStart.get()
            endtime= self.app.ltEnd.get()        
            ulab= ui['ulab'].get(); key= ui['key'].get()
            acol= ui['acol'].get(); alt= ui['alt'].get()
            df= self.DS.sqlExprFilter(uid, starttime, endtime, ulab, key, acol, alt)
            if acol==tt.tree['columns'][2]:
                acol=tt.tree['columns'][3]
                ui['acol'].set(acol)
            tt.delValues()
            tt.addValues(df)
            (jn,df)= tt.getHints(key)
            if len(jn)==1:
                #self.ui['key'].set(jn[0])
                tt.tree.item(jn[0], open=TRUE)            

    def onHints(self):
        tt= self.tt
        tt.delValues()
        tt.addValues(self.fetch)
        key= self.ui['key'].get()
        (jn,df)= tt.getHints(key)
        if len(jn)!=0:
            tt.delValues()
            tt.addValues(df)
        if len(jn)==1:
            #self.ui['key'].set(jn[0])
            tt.tree.item(jn[0], open=TRUE)


    def NumInput(self, event=None):
        if self.onNumKey is None:
            self.onNumKey= PopupNumKey(self, self.user)
        else:
            self.onNumKey.show()

    def NUMhide(self):
        if self.onNumKey is not None:
            self.onNumKey.hide()

    def isValid(self,event=None):
        DV_key= ''		
        ui= self.ui
        self.NUMhide()
        if ui['uid'] is None:
            key= ui['key'].get()
            key= key.strip()
            try:
                if len(key)==10:
                    uid= re.search(r'^00(\d{8})',key).group(1)
                    ui['uid']= uid
                    self.onAll()
                    self.app.uop= self.DS.sqlDBuser(uid)
                    if self.app.uop is None:
                        self.app.uop= uid
                    self.app.filter.config(text=self.app.uop)
                    self.showTable()
                    key= ''
                else:
                    key=DV_key
            except:
                key= DV_key
            ui['key'].set(key)
            self.user.focus_force()
        else:
            self.app.after(100, self.getIDtool)
            
    # XXX II_tools ok
    def getIDtool(self):
        tree= self.tt.tree
        tt= self.tt
        key= self.ui['key'].get()
        key= key.strip()
        try:
            if len(key)==10:
                toolid= re.search(r'^00(\d{8})',key).group(1)
                #new revised on Nov 2016
                (df,toolclass,dfid)= self.DS.sqlToolInfo(toolid)
                print "read tool id %s" % dfid
                aix= self.ui['aix']
                self.ui['acol'].set(tree["columns"][aix])
                self.fetch= df
                tt.delValues()
                tt.addValues(df)
                self.ui['key'].set(toolid)
                self.ui['alt'].set('')
                if toolclass is not None:
                    self.ui['fbtn'].config(text= u"類別 %s" % toolclass)
                    self.ui['dfid']= dfid
                else:
                    self.ui['fbtn'].config(text= "")
                    self.ui['dfid']= u""

                if toolclass is not None:
                    item_so= str(df[0][0])
                    tree.see(item_so)
                    tree.selection_add((item_so,))
                    self.app.after(100, self.onTool)
                
                if dfid is None:
                    self.ui['key'].set("")                    
     
        except:
            self.ui['key'].set("")
            print (df,toolclass,dfid)
            print "getidtool error"

        self.user.focus_force()   

    #enter measurement by II_tools id 
    def onTool(self):
        tree= self.tt.tree
        self.NUMhide()
        if self.isRowDialog:        
            return
        item = tree.selection()
        print "item: {}".format(item)  
        if len(item)!=1:
            print "not select yet!"
            return

        try:
            item = tree.selection()[0]
            selvals= tree.item(item,"values")
            if selvals!='':
                #log time to console
                lt= time.localtime(time.time())
                print "#%02d:%02d#  tool starts record" % (lt[3:5])               
                print "df match ID %s" % self.ui['dfid']
                fetch=self.DS.sqlToolEnquiry(selvals,self.ui['dfid'])
                fetch= {'spec':fetch,'values':selvals}
                self.app.m.set(1)
                self.app.onUpdate() 
                self.isRowDialog= True
                spec_unitkey= self.app.UNIT.index(fetch['spec'][2])
                spec_convert= self.app._convert[spec_unitkey]                 
                
                vkey= self.verifybox(fetch['spec'], spec_convert)
                if vkey>0:
                    self.app.bx.set(vkey)
                    self.app.onBoxchange()
                    self.isRowDialog= False
                    self.ui['key'].set("")                     
                    print "request to change Box..."
                    return
                
                #pass app.tframe.bi image to Dialog 
                toolid= self.ui['key'].get()
                (cardid, sid, unit)= (self.ui['uid'], fetch['values'][0],fetch['spec'][2])
                (fetch, toolid)= winDialog(self.bi,fetch,tool=toolid).result
                tool= toolid
                if fetch is None:
                    self.ui['key'].set("")                    
                    self.errmsgTOOL("tool enquiry cancel")  
            else:
                tool= None
        except:
            tool= None
            #self.ui['key'].set("")               
            #self.errmsgTOOL("tool enquiry cancel")  
        self.isRowDialog= False

        try:
            if tool is None:
                return
            isCheck= self.DS.sqlToolCheck(tool, fetch['spec'])
            isCheck= (isCheck[0]=='Ok')
            if not isCheck:
                self.ui['key'].set("")                 
                self.errmsgTOOL("Tool not available or suitable!")                
                return
            spec_unitkey= self.app.UNIT.index(fetch['spec'][2])    
            self.app.u.set(spec_unitkey)
            self.app.m.set(1)
            self.app.onUpdate()
            self.isRowDialog= True
            time.sleep(0.2)
            #log date to measurement
            lt="%04d-%02d-%02d" % (lt[0:3])         
            remark= self.DS.sqlDBremark()
            #pass app.tframe.bi image to Dialog
            result= toolDialog(self.bi,fetch,self.app.rCH,tool,remark).result
            (fetch, tool, VALUES, RET, REM)= result
            if RET=='PASS':
                RET= u'合格'
                self.showPASS()
            elif RET=='FAIL':
                RET= u'不合格'
                self.showFAIL()
            median= VALUES[3]
            self.DS.sqlToolUpdate(sid,median,unit,RET,tool,lt,cardid,REM)
            self.ui['key'].set('')
            self.ui['fbtn'].config(text= "")
            self.onFilter()
        except:
            self.ui['key'].set("")               
            self.errmsgTOOL("tool measurement are not completed!")   
        self.isRowDialog= False   


    def verifybox(self, spec, spec_convert):
        print spec
        upper_limit= float(spec[1]) / spec_convert
        
        vkey= 3
        #if upper_limit<=4.1:
        #    vkey= 2
        if upper_limit<=2.1:
            vkey= 2             # 1 debugging using 10Nm 
        if upper_limit > 20.0:
            vkey= -1
            print "spec higher than torque limit 20Nm!"
            
        if vkey==int(self.app.rCH.get('mbBX')):
            self.app.bx.set(vkey)
            vkey= 0
        return vkey         #request when > 0
    
    def showTable(self):
        # init a treeview with dual scrollbars
        ui= self.ui
        ui['opts'].grid(row=1, column=4, padx= 3, sticky='w')
        ui['allbtn'].grid(row=1, column=0, sticky='w', padx=3, in_ = self.fh)
        ui['fbtn'].grid(row=1, column=5, padx=3, in_ = self.fh)
        ui['ulab'].set(ui['kcol'])
        self.tt.grid(self.bi)   #refer app.tframe.bi image

    def winfo_xy(self,fh_xy=(200,-140)):
        return (self.fh.winfo_rootx()+fh_xy[0], self.fh.winfo_rooty()+fh_xy[1])

    def errmsgTOOL(self,msg='Tool Error'):
        #popup message windows for around 1.6sec.
        winMessage(self.app.reading,{'winfo_xy': self.winfo_xy(),'title':'Tool',
                                     'message':msg,'fgcolor':'brown'})
        print msg

    def showBOX(self,msg=''):
        """ Box pluging alert """        
        if msg!='':
            winMessage(self.app.reading,{'winfo_xy':self.winfo_xy(),
                        'title':'Box Message', 'message':msg,'fgcolor':'orange'})

    def showPASS(self,msg='PASS'):
        winMessage(self.app.reading,{'winfo_xy':self.winfo_xy(),'title':'Result',
                                     'message':msg,'fgcolor':'green'})

    def showFAIL(self,msg='FAIL'):
        winMessage(self.app.reading,{'winfo_xy':self.winfo_xy(),'title':'Result',
                                     'message':msg,'fgcolor':'red'})


    #enter measurement
    def onRowClick(self):
        tree= self.tt.tree
        self.NUMhide()
        if not self.isRowDialog:
            try:
                item = tree.selection()[0]
                print "item: {}".format(item)
                selvals= tree.item(item,"values")
                if selvals!='':
                    #log time to console
                    lt= time.localtime(time.time())
                    print "#%02d:%02d#  tool starts record" % (lt[3:5])               
                    fetch=self.DS.sqlToolEnquiry0(selvals)
                    fetch= {'spec':fetch,'values':selvals}
                    self.app.m.set(1)
                    self.app.onUpdate() 
                    self.isRowDialog= True
                    spec_unitkey= self.app.UNIT.index(fetch['spec'][2])
                    spec_convert= self.app._convert[spec_unitkey]                 
                    
                    vkey= self.verifybox(fetch['spec'], spec_convert)
                    if vkey>0:
                        self.app.bx.set(vkey)
                        self.app.onBoxchange()
                        self.isRowDialog= False
                        print "request to change Box..."
                        return
                    
                    #pass app.tframe.bi image to Dialog 
                    (cardid, sid, unit)= (self.ui['uid'], fetch['values'][0],fetch['spec'][2])
                    (fetch, tool)= winDialog(self.bi,fetch).result
                    if fetch is None:
                        self.errmsgTOOL("tool enquiry cancel")  
                else:
                    tool= None
            except:
                tool= None
                self.errmsgTOOL("tool enquiry cancel")  
            self.isRowDialog= False
            try:
                if tool is None:
                    return
                isCheck= self.DS.sqlToolCheck(tool, fetch['spec'])
                isCheck= (isCheck[0]=='Ok')
                if not isCheck:
                    self.errmsgTOOL("Tool not available or suitable!")                
                    return
                spec_unitkey= self.app.UNIT.index(fetch['spec'][2])    
                self.app.u.set(spec_unitkey)
                self.app.m.set(1)
                self.app.onUpdate()
                self.isRowDialog= True
                time.sleep(0.2)
                #log date to measurement
                lt="%04d-%02d-%02d" % (lt[0:3])         
                remark= self.DS.sqlDBremark()
                #pass app.tframe.bi image to Dialog
                result= toolDialog(self.bi,fetch,self.app.rCH,tool,remark).result
                (fetch, tool, VALUES, RET, REM)= result
                if RET=='PASS':
                    RET= u'合格'
                    self.showPASS()
                elif RET=='FAIL':
                    RET= u'不合格'
                    self.showFAIL()
                median= VALUES[3]
                self.DS.sqlToolUpdate(sid,median,unit,RET,tool,lt,cardid,REM)
                self.onFilter()
            except:
                self.errmsgTOOL("tool measurement are not completed!")   
            self.isRowDialog= False   

#end of app
