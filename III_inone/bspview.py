#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Torque App by TRiO
'''
import sys, atexit
from II_tkForm2 import winApp
import pypyodbc, time
import III_inone


def isRaspi():
    return (sys.platform=='linux2')     #'win32' or 'linux2'

def datetime():
    return time.strftime('%y-%m-%d %H:%M:%S', time.localtime(time.time())) 
    
def isToolOpen():
    try:
        import redis                  
        r= redis.Redis('localhost')     #localDB enquiry
        return (r.get('mbOpen')== 'True')    
    except:
        print "localDB not ready"
        return False

def ToolClose():
    try:
        import redis                  
        r= redis.Redis('localhost')     #localDB enquiry 
        r.set('mbQuit', True)
        r.set('mbOpen', False)        
    except:
        print "abnormal during closing" 
    time.sleep(1.2)


class sqlConn():
    """MS SQL interface in cloud V2.0 """  # XXX ok 
    exp_Dsn2= "DSN=SQLEXPRESS; UID=sa; PWD=pi2; DATABASE=Engg"
    exp_Str2= "DRIVER={SQL Server};SERVER=localhost\SQLEXPRESS;UID=sa;PWD=pi2;DATABASE=Engg"
    tmc_Dsn2= "DSN=sqlDS; UID=tmicsuser; PWD=123456; DATABASE=Engg"
    tmc_Str2= "DRIVER={SQL Server};SERVER=192.168.100.23;UID=tmicsuser;PWD=123456;DATABASE=Engg"
    tmc_Dsn= "DSN=sqlDS; UID=Engg; PWD=engg0222; DATABASE=Engg"
    tmc_Str= "DRIVER={SQL Server};SERVER=192.168.33.93;UID=Engg;PWD=engg0222;DATABASE=Engg"

    Electric_head= [u'Id', u'JobNo', u'Model', u'ProcessCode', u'ProcessName',
                    u'NoticeStaffName', u'NoticeTime', u'Workshop', u'Line', u'State']
    Electric_state= [u'None',u'待批核',u'已批核']
    #displaycolumns on table 'Tools'
    DSindex= (1,2,4,8,9,6,7,16,3,14)

    def __init__(self, isTMC=True):
        self.isTMC= isTMC
        self.isRaspi= isRaspi()
        self.stdlogNew()
        if self.isTMC:
            self.connStr= self.tmc_Dsn if self.isRaspi else self.tmc_Str
        else:
            self.connStr= self.exp_Dsn2 if self.isRaspi else self.exp_Str2
        try:
            print '\nTools odbc data source...'
            print '--- connected on %s ---\n' % datetime()
            pypyodbc.connection_timeout= 6
            self.connDB= pypyodbc.connect(self.connStr,timeout=4)
            self.connDB.timeout= 4
            self.cur= self.connDB.cursor()
            atexit.register(self.close)

        except:
            self.cur= None
            self.connDB= None 
            print 'not connect\n'

    def stdlogNew(self):
        with open("log/stdlog.txt", 'w') as fp:
            fp.write("Welcome, III inone starts here @2020\n\n")

    def stdlogOpen(self):
        sys.stdout = open("log/stdlog.txt", 'a')

    def stdlogClose(self):
        sys.stdout.close()

    def isConn(self):
        return  self.connDB is not None and self.cur is not None

    def close(self):
        try:
            if self.connDB is not None:
                self.connDB.close()
            print '--- Data Source closed on %s ---' % datetime()
        except:
            print 'cannot close on DS'

    def sqlDB(self):
        colHeader= self.sqlDBheading()
        fetch= self.sqlExpr(uid= '08149242')    #08149242
        return (colHeader,fetch)

    def sqlExpr(self,uid, stime='',etime=''):
        fetch= None
        if self.isConn():
            ###II+ shorten waiting time by one-day period 
            if stime=='':
                lt= time.localtime(time.time())
                lt="%04d-%02d-%02d" % (lt[0:3])
                stime, etime = (lt, lt)
            try:
                if self.isTMC:
                    return self.sqlCardEnquiry(uid,'',stime,etime)
                else:
                    SQL= "SELECT * FROM Tools"
                    self.cur.execute(SQL)
                    fetch= self.cur.fetchall()
            except:
                print 'cursor fetching error on express'
        return self.patch(fetch)

    #prototype of tmics sorting
    def sqlExprFilter(self,uid,starttime,endtime,khead,key,fhead,fsel):
        if self.isConn():
            try:
                if fhead==u'State':
                    vstate= dict(zip(self.Electric_state,('0','1','2')))
                    fsel= vstate[fsel]
                elif fhead==u'Workshop':
                    fsel= fsel[0:3]
                if self.isTMC:
                    return self.sqlCardFilter(uid,starttime,endtime,khead,key,fhead,fsel)
                else:
                    SQL= "SELECT * FROM Tools "
                    if key=='':
                        SQL+= "WHERE %s = ? " % fhead
                        pars= [fsel]
                    elif fsel=='':
                        SQL+= "WHERE %s = ? " % khead
                        pars= [key]                        
                    else:
                        SQL+= "WHERE %s = ? AND %s = ? " % (khead,fhead) 
                        pars= [key,fsel]
                    self.cur.execute(SQL, pars)
                    fetch= self.cur.fetchall()
                    return self.patch(fetch)
            except:
                print 'cursor fetching error on enquiry'
        return []

    #implement tmics filtering
    def sqlCardFilter(self,cardid,starttime,endtime,kid,key,fid,fsel):
        fetch= None
        if self.isConn() and self.isTMC:
            try:
                fetch= []
                SQL= "exec Engg.Proc_Electric_QueryUnProofInfo "
                SQL+= "? , ? , ? , ? , ? , ? , ? "
                pars= [cardid,starttime,endtime,kid,key,fid,fsel]
                #print pars when debug
                self.cur.execute(SQL, pars)
                fetch= self.cur.fetchall()
                return self.patch(fetch)
            except:
                print 'cursor fetching error on enquiry'
        return self.patch(fetch)

    def sqlCardEnquiry(self,cardid,jobno,starttime,endtime):
    #DS.sqlCardEnquiry('08149640','10007705','2015-07-01','2015-07-31')
        fetch= None
        if self.isConn() and self.isTMC:
            try:
                fetch= []
                SQL= "exec Engg.Proc_Electric_QueryUnProofInfoByCard "
                SQL+= "? , ? , ? , ? "
                pars= [cardid,jobno,starttime,endtime]
                self.cur.execute(SQL,pars)
                fetch= self.cur.fetchall()
                return self.patch(fetch)
            except:
                print 'cursor fetching error on enquiry'
        return self.patch(fetch)

    def sqlToolInfo(self,tool):
    #DS.sqlToolInfo('01207923')
        fetch= None
        if self.isConn() and self.isTMC:
            try:
                fetch= []
                SQL= "exec Engg.Proc_Electric_QueryUnProofInfoByElectricCardId ? "
                pars= [tool]
                self.cur.execute(SQL,pars)
                fetch= self.cur.fetchall()
                return self.patch2(fetch)
            except:
                print 'cursor fetching error on tool id'
        return self.patch2(fetch)

    def patch2(self,fetch):
        if fetch is not None:
            table= []               ;#format in [(row0),(row1),...]
            if len(fetch):
                for select in fetch:
                    flag= str(select[-3])
                    flag= int(flag) if flag in ('1','2') else 0
                    #table.append(select[0:-2]+(self.Electric_state[flag],select[-1]))
                    table.append(select[0:-3]+(self.Electric_state[flag],))
                return (table,select[-2],select[-1])

        return (None,None,None)


    def patch(self,fetch):
        if fetch is not None:
            table= []               ;#format in [(row0),(row1),...]
            if len(fetch):
                idx= len(fetch[0])-1
                for select in fetch:
                    flag= str(select[idx])
                    flag= int(flag) if flag in ('1','2') else 0
                    table.append(select[0:idx]+(self.Electric_state[flag],))
                return table
        return fetch     

    def sqlDBuser(self,cardid):
        user= None
        try:
            SQL="exec Engg.Proc_Electric_QueryUserNameByCard ?"
            pars= [cardid]
            self.cur.execute(SQL, pars)
            user= self.cur.fetchone()
            return user
        except:
            print 'cursor fetching error on user'

    def sqlDBheading(self):
        fetch= None
        if self.isConn():
            try:            
                if self.isTMC:
                    SQL= "exec Engg.Proc_Electric_QueryTableHead "
                    self.cur.execute(SQL)
                    rows= self.cur.fetchall()
                    fetch= list( (x[0] for x in rows) )
                    fetch= [fetch[i] for i in self.DSindex]
                else:
                    SQL= """
                    SELECT COLUMN_NAME 
                    FROM INFORMATION_SCHEMA.COLUMNS
                    WHERE TABLE_NAME = 'Tools'
                    """
                    self.cur.execute(SQL)
                    rows= self.cur.fetchall()
                    fetch= list( (x[0] for x in rows) )
            except:
                print 'cursor fetching error on heading'
        if fetch is None:
            print 'column using DEBUG heading'
            fetch=  self.Electric_head
        return fetch     

    def sqlDBremark(self):
        fetch= None
        if self.isConn():
            try:
                if self.isTMC:
                    SQL= "exec Engg.Proc_Electric_QueryProofFailRemark "
                    self.cur.execute(SQL)
                    rows= self.cur.fetchall()
                    fetch= list( (x[0] for x in rows) )
                    fetch= ('',)+tuple(fetch)
            except:
                print 'cursor fetching error on remark'
        if fetch is None:
            print 'tool remark using DEBUG list'            
            fetch= (u'',u'電批壞',u'扭力設定值不在範圍內',u'未設定相關扭力值',u'其它')
        return fetch

    def sqlToolEnquiry0(self,selvals):
        fetch= None
        if self.isConn():
            try:            
                if self.isTMC:
                    #params: model,processcode,processname
                    SQL= "exec Engg.Proc_Electric_QueryProofInfo "
                    SQL+= "? , ? , ? "
                    pars= selvals[2:5]
                    self.cur.execute(SQL, pars)
                    #0:MinStrength,1:MaxStrength,2:StrengthUnit
                    row= self.cur.fetchone()
                    row2= 'Kgf.cm' if row[2]==u'kgf.cm' else row[2]
                    fetch= row[0:2]+(row2,)
                else:
                    fetch= (0.97, 1.03, 'N.m') 
            except:
                print 'cursor fetching error on spec'
                fetch= None
        return fetch

    def sqlToolEnquiry(self,selvals, dfid):
        fetch= None
        if self.isConn():
            try:
                if self.isTMC:
                    #params: model,processcode,processname
                    SQL= "exec Engg.Proc_Electric_QueryProofInfo_1 "
                    SQL+= "? , ? , ? , ?"
                    pars= tuple(list(selvals[2:5]) + [dfid])
                    self.cur.execute(SQL, pars)
                    #0:MinStrength,1:MaxStrength,2:StrengthUnit
                    row= self.cur.fetchone()
                    row2= 'Kgf.cm' if row[2]==u'kgf.cm' else row[2]
                    fetch= row[0:2]+(row2,)
                else:
                    fetch= (0.97, 1.03, 'N.m') 

            except:
                print 'cursor fetching error on spec'
                fetch= None

        return fetch 
  
    def sqlToolCheck(self, ecid, spec):
        #DS.sqlToolCheck('01160262',['0.3','2.5','N.m'])
        if self.isConn() and self.isTMC:
            try:
                # selvals for model,processcode,processname
                SQL= "exec Engg.Proc_Electric_CheckElectricMatch "
                SQL+= " ? , ? , ? , ? "
                (minst, maxst, stunit)= tuple(spec[0:3])
                pars= [ecid, minst, maxst, stunit]
                print pars
                self.cur.execute(SQL, pars)
                return self.cur.fetchall()[0]
            except:
                print 'cursor fetching error on tool id'
                return('Fail',)
        else:
            return ('Ok',)

    def sqlToolUpdate(self,ecid,median,unit,RET,tool,lt,cardid,REM):
        #log record:('2', '5.76', u'Kgf.cm', u'合格', '01160262', '2015-08-29', '08149242', u'No')
        if self.isConn() and self.isTMC:
            try: 
                SQL= "exec Engg.Proc_Electric_UpdateProofInfo "
                SQL+= " ?, ?, ?, ?, ?, ?, ?, ? "
                pars= [ecid,median,unit,RET,tool,lt,cardid,REM]
                print pars
                self.cur.execute(SQL, pars)
                SQL1= "select * from TMICS.tmicsuser.ElectricScrewdriver_NoticePEProofStrength "
                SQL1+= "where Id=?"
                self.cur.commit()
                pars= [ecid]
                self.cur.execute(SQL1,pars)        
                #rows= self.cur.fetchall()
                return True
            except:
                print 'cursor fetching error on update of test-data'    
                return False
        else:
            return True         #NULL state


def test(DS):
    toolid='07114517'
    (df,toolclass,dfid)= DS.sqlToolInfo(toolid)
    print(len(df))
    return df

def main(debug=False):
    global inone, app
    """ Torque Meter """        # XXX ok 
    if isToolOpen():
        print 'TOOL is not the first instance'
        time.sleep(2)
        return 0
    
    #SPI32 setup
    try:
        print "\n\n"
        inone = III_inone.Channel()
        r= inone.redis 
        time.sleep(0.4)
        
    except:
        r= None
        print "channel not ready!"

    if r is None:
        ToolClose()
        return

    #run application at once time
    if not debug:
        try:
            while(r.get('mbNew')=='True'):
                #sqlDS setup
                DS= sqlConn()
                DS.stdlogOpen()
                #app setup
                app= winApp(DS,r)  
                app.start()
        except:
            print "app/sqlDS stop appoint to have error!"
    
        finally:
            inone.set_shutdown()
            ToolClose()
    
    else:
        #sqlDS setup
        DS= sqlConn()
        DS.stdlogOpen()
        #app setup
        time.sleep(1.2)
        app= winApp(DS,r)
        app.start()
        inone.set_shutdown()
        ToolClose()


if __name__ == "__main__":
    #DS= sqlConn(True); test(DS)
    main()
