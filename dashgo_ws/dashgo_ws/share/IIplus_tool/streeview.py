# -*- coding: utf-8 -*-
"""
*** treeview usage ***
    tree["columns"]=("one","two")
    tree.column("one", width=100 )
    tree.heading("one", text="coulmn A")
    tree.insert("" , 0,    text="Line 1", values=("1A","1b"))
    id2 = tree.insert("", 1, "dir2", text="Dir 2")
    tree.insert(id2, "end", "dir 2", text="sub dir 2", values=("2A","2B"))
"""
from Tkinter import Tk
from bsp_data import colhead as columns
from bspview import sqlConn
import ttk

#from bsp_data import fetch as sqlfetch
DS= sqlConn(isTMC=True)
sqlfetch= DS.sqlExpr('')
root = Tk()
tree = ttk.Treeview(root)


def getIID(jn=''):
    if jn=='':
        child= sum([tree.get_children(p) for p in tree.get_children('')],())
    else:
        child= tree.get_children(jn)
    return child 

def getValues(iid=''):
    values= []
    if iid=='':
        for c in getIID():
            values.append(tree.item(c)['values'])
    else:
        values= tree.item(iid)['values']
    return values

def addValues(self,fetch):
    #set-up icons by JobNo and insert option: open=TRUE
    tree= self
    if fetch is not None:
        jicon= [s[1] for s in fetch]
        jicon= list(sorted(set(jicon)))
        wait= u'\u5f85\u6279\u6838'
        for jn in jicon:
            jwait= [item[1]==jn and item[9]==wait for item in fetch]
            jtext= '%s  {%d wait}' % (jn, sum(jwait))
            tree.insert("",1,jn,text=jtext)
        for item in fetch:
            item= list(item)
            item[7].strip()
            item[7]= "%s%s" % (item[7].strip(), item[8].strip())
            jn= item[1]; iid= item[0]; item[8]= ''
            #print(jn,"end",iid, item[2], item) when debug
            tree.insert(jn,"end",iid, text=item[2], values=item)

def sortby(self, acol, descending):
    """sort tree contents when a column header is clicked on"""
    tree= self
    for jn in tree.get_children(''):
        items = [(tree.set(c, acol), c) for c in self.getIID(jn)]
        items.sort(reverse=descending)
        for (ix, item) in enumerate(items):
            tree.move(item[1], jn, ix)
    # switch the heading so it will sort in the opposite direction
    tree.heading(acol, command=lambda c=acol: self.sortby(c, int(not descending)))


def onRowSelect(event):
    item = tree.selection()[0]      #iid 
    fetch= tree.item(item,"values")
    print item
    print fetch


if __name__ == "__main__":
    tcwidth= (100,100,100,100) + (100,100,125) + (100,40,75)
    tcdisplay= (8,3,4,6,7,9)

    tree["columns"]= tuple(columns)             #set column header
    tree.configure(displaycolumns=tcdisplay)    #show required columns 
    tree.configure(height=50,padding=(5,2)) 
    for (i,col) in enumerate(columns):
        tree.column(col, width= tcwidth[i])     #set column width
        tree.heading(col, text=col, command=lambda c=col: sortby(c, 0))
    tree.column('#0', width=320)
    tree.heading('#0', text='%s/%s' % tuple(columns[1:3]))
    tree.heading('#0', command=lambda c=columns[2]: sortby(c, 0))    
    tree.bind("<<TreeviewSelect>>", onRowSelect)  
    
    addValues(tree,sqlfetch)
    tree.pack()
    item= getIID()
    tree.see(item[0])
    tree.selection_add(item[:1])
    root.mainloop()
