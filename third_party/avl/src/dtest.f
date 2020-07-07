
      PROGRAM DTEST
C---------------------------------------------------------------------------
C     Checks stability derivatives by finite differencing.
C     Usage: 
C             % coef xxx yyy
C
C     where "xxx", "yyy" are AVL stability derivative output filenames.
C
C---------------------------------------------------------------------------
      IMPLICIT REAL(A-H,O-Z)

      CHARACTER*80 ARGP1, ARGP2
      CHARACTER*128 LINE
C
      PI = 4.0*ATAN(1.0)
C
C--------------------------------------
C
 1000 FORMAT(A)
C
C---- get Unix command arguments
      CALL GETARG(1,ARGP1)
      CALL GETARG(2,ARGP2)
C
C-----------------------------------------------------------------
      OPEN(1,FILE=ARGP1,STATUS='OLD')
      ILINE = 0
C
C==================================================================
C---- top of file-line reading loop
 10   CONTINUE
      READ(1,1000,END=11) LINE
      ILINE = ILINE + 1
      IF(INDEX('#!',LINE(1:1)).NE.0) GO TO 10
C
      CALL GETVAL(LINE,'Alpha =',ADEG)
      CALL GETVAL(LINE,'Beta  =',BDEG)
      a = ADEG*PI/180.0
      b = BDEG*PI/180.0
C
      CALL GETVAL(LINE,'p''b/2V =',p)
      CALL GETVAL(LINE,  'qc/2V =',q)
      CALL GETVAL(LINE,'r''b/2V =',r)

      CALL GETVAL(LINE,'Cl''tot =',Cr)
      CALL GETVAL(LINE,  'Cmtot =',Cm)
      CALL GETVAL(LINE,'Cn''tot =',Cn)

      CALL GETVAL(LINE,'CYtot =',CY)
      CALL GETVAL(LINE,'CLtot =',CL)
      CALL GETVAL(LINE,'CDtot =',CD)
      CALL GETVAL(LINE,'CDff  =',CDff)
      CALL GETVAL(LINE,'    e =', e)

      CALL GETVAL(LINE,'flap            =',d1)
      CALL GETVAL(LINE,'aileron         =',d2)
      CALL GETVAL(LINE,'elevator        =',d3)
      CALL GETVAL(LINE,'rudder          =',d4)

      CALL GETVAL(LINE,'CLa =',CLa)
      CALL GETVAL(LINE,'CYa =',CYa)
      CALL GETVAL(LINE,'Cla =',Cra)
      CALL GETVAL(LINE,'Cma =',Cma)
      CALL GETVAL(LINE,'Cna =',Cna)
                
      CALL GETVAL(LINE,'CLb =',CLb)
      CALL GETVAL(LINE,'CYb =',CYb)
      CALL GETVAL(LINE,'Clb =',Crb)
      CALL GETVAL(LINE,'Cmb =',Cmb)
      CALL GETVAL(LINE,'Cnb =',Cnb)
                
      CALL GETVAL(LINE,'CLp =',CLp)
      CALL GETVAL(LINE,'CYp =',CYp)
      CALL GETVAL(LINE,'Clp =',Crp)
      CALL GETVAL(LINE,'Cmp =',Cmp)
      CALL GETVAL(LINE,'Cnp =',Cnp)
                
      CALL GETVAL(LINE,'CLq =',CLq)
      CALL GETVAL(LINE,'CYq =',CYq)
      CALL GETVAL(LINE,'Clq =',Crq)
      CALL GETVAL(LINE,'Cmq =',Cmq)
      CALL GETVAL(LINE,'Cnq =',Cnq)
                
      CALL GETVAL(LINE,'CLr =',CLr)
      CALL GETVAL(LINE,'CYr =',CYr)
      CALL GETVAL(LINE,'Clr =',Crr)
      CALL GETVAL(LINE,'Cmr =',Cmr)
      CALL GETVAL(LINE,'Cnr =',Cnr)

      CALL GETVAL(LINE,'CLd1 =',CLd1)
      CALL GETVAL(LINE,'CYd1 =',CYd1)
      CALL GETVAL(LINE,'Cld1 =',Crd1)
      CALL GETVAL(LINE,'Cmd1 =',Cmd1)
      CALL GETVAL(LINE,'Cnd1 =',Cnd1)
      CALL GETVAL(LINE,'CDffd1 =',CDffd1)
      CALL GETVAL(LINE,'ed1 =',ed1)

      CALL GETVAL(LINE,'CLd2 =',CLd2)
      CALL GETVAL(LINE,'CYd2 =',CYd2)
      CALL GETVAL(LINE,'Cld2 =',Crd2)
      CALL GETVAL(LINE,'Cmd2 =',Cmd2)
      CALL GETVAL(LINE,'Cnd2 =',Cnd2)
      CALL GETVAL(LINE,'CDffd2 =',CDffd2)
      CALL GETVAL(LINE,'ed2 =',ed2)

      CALL GETVAL(LINE,'CLd3 =',CLd3)
      CALL GETVAL(LINE,'CYd3 =',CYd3)
      CALL GETVAL(LINE,'Cld3 =',Crd3)
      CALL GETVAL(LINE,'Cmd3 =',Cmd3)
      CALL GETVAL(LINE,'Cnd3 =',Cnd3)
      CALL GETVAL(LINE,'CDffd3 =',CDffd3)
      CALL GETVAL(LINE,'ed3 =',ed3)

      CALL GETVAL(LINE,'CLd4 =',CLd4)
      CALL GETVAL(LINE,'CYd4 =',CYd4)
      CALL GETVAL(LINE,'Cld4 =',Crd4)
      CALL GETVAL(LINE,'Cmd4 =',Cmd4)
      CALL GETVAL(LINE,'Cnd4 =',Cnd4)
      CALL GETVAL(LINE,'CDffd4 =',CDffd4)
      CALL GETVAL(LINE,'ed4 =',ed4)
      GO TO 10
C
C=============================================
C
 11   CONTINUE
      CLOSE(1)
C
C-----------------------------------------------------------------
      OPEN(2,FILE=ARGP2,STATUS='OLD')
      ILINE = 0
C
C==================================================================
C---- top of file-line reading loop
 20   CONTINUE
      READ(2,1000,END=21) LINE
      ILINE = ILINE + 1
      IF(INDEX('#!',LINE(1:1)).NE.0) GO TO 20
C
      CALL GETVAL(LINE,'Alpha =',ADEG)
      CALL GETVAL(LINE,'Beta  =',BDEG)
      a$ = ADEG*PI/180.0
      b$ = BDEG*PI/180.0
C
      CALL GETVAL(LINE,'p''b/2V =',p$)
      CALL GETVAL(LINE,  'qc/2V =',q$)
      CALL GETVAL(LINE,'r''b/2V =',r$)

      CALL GETVAL(LINE,'Cl''tot =',Cr$)
      CALL GETVAL(LINE,  'Cmtot =',Cm$)
      CALL GETVAL(LINE,'Cn''tot =',Cn$)

      CALL GETVAL(LINE,'CYtot =',CY$)
      CALL GETVAL(LINE,'CLtot =',CL$)
      CALL GETVAL(LINE,'CDtot =',CD$)
      CALL GETVAL(LINE,'CDff  =',CDff$)
      CALL GETVAL(LINE,'    e =', e$)

      CALL GETVAL(LINE,'flap            =',d1$)
      CALL GETVAL(LINE,'aileron         =',d2$)
      CALL GETVAL(LINE,'elevator        =',d3$)
      CALL GETVAL(LINE,'rudder          =',d4$)

      CALL GETVAL(LINE,'CLa =',CLa$)
      CALL GETVAL(LINE,'CYa =',CYa$)
      CALL GETVAL(LINE,'Cla =',Cra$)
      CALL GETVAL(LINE,'Cma =',Cma$)
      CALL GETVAL(LINE,'Cna =',Cna$)
                
      CALL GETVAL(LINE,'CLb =',CLb$)
      CALL GETVAL(LINE,'CYb =',CYb$)
      CALL GETVAL(LINE,'Clb =',Crb$)
      CALL GETVAL(LINE,'Cmb =',Cmb$)
      CALL GETVAL(LINE,'Cnb =',Cnb$)
                
      CALL GETVAL(LINE,'CLp =',CLp$)
      CALL GETVAL(LINE,'CYp =',CYp$)
      CALL GETVAL(LINE,'Clp =',Crp$)
      CALL GETVAL(LINE,'Cmp =',Cmp$)
      CALL GETVAL(LINE,'Cnp =',Cnp$)
                
      CALL GETVAL(LINE,'CLq =',CLq$)
      CALL GETVAL(LINE,'CYq =',CYq$)
      CALL GETVAL(LINE,'Clq =',Crq$)
      CALL GETVAL(LINE,'Cmq =',Cmq$)
      CALL GETVAL(LINE,'Cnq =',Cnq$)
                
      CALL GETVAL(LINE,'CLr =',CLr$)
      CALL GETVAL(LINE,'CYr =',CYr$)
      CALL GETVAL(LINE,'Clr =',Crr$)
      CALL GETVAL(LINE,'Cmr =',Cmr$)
      CALL GETVAL(LINE,'Cnr =',Cnr$)

      CALL GETVAL(LINE,'CLd1 =',CLd1$)
      CALL GETVAL(LINE,'CYd1 =',CYd1$)
      CALL GETVAL(LINE,'Cld1 =',Crd1$)
      CALL GETVAL(LINE,'Cmd1 =',Cmd1$)
      CALL GETVAL(LINE,'Cnd1 =',Cnd1$)
      CALL GETVAL(LINE,'CDffd1 =',CDffd1$)
      CALL GETVAL(LINE,'ed1 =',ed1$)

      CALL GETVAL(LINE,'CLd2 =',CLd2$)
      CALL GETVAL(LINE,'CYd2 =',CYd2$)
      CALL GETVAL(LINE,'Cld2 =',Crd2$)
      CALL GETVAL(LINE,'Cmd2 =',Cmd2$)
      CALL GETVAL(LINE,'Cnd2 =',Cnd2$)
      CALL GETVAL(LINE,'CDffd2 =',CDffd2$)
      CALL GETVAL(LINE,'ed2 =',ed2$)

      CALL GETVAL(LINE,'CLd3 =',CLd3$)
      CALL GETVAL(LINE,'CYd3 =',CYd3$)
      CALL GETVAL(LINE,'Cld3 =',Crd3$)
      CALL GETVAL(LINE,'Cmd3 =',Cmd3$)
      CALL GETVAL(LINE,'Cnd3 =',Cnd3$)
      CALL GETVAL(LINE,'CDffd3 =',CDffd3$)
      CALL GETVAL(LINE,'ed3 =',ed3$)

      CALL GETVAL(LINE,'CLd4 =',CLd4$)
      CALL GETVAL(LINE,'CYd4 =',CYd4$)
      CALL GETVAL(LINE,'Cld4 =',Crd4$)
      CALL GETVAL(LINE,'Cmd4 =',Cmd4$)
      CALL GETVAL(LINE,'Cnd4 =',Cnd4$)
      CALL GETVAL(LINE,'CDffd4 =',CDffd4$)
      CALL GETVAL(LINE,'ed4 =',ed4$)
      GO TO 20
C
C=============================================
C
 21   CONTINUE
      CLOSE(2)
C
      da = a$ - a
      db = b$ - b
      dp = p$ - p
      dq = q$ - q
      dr = r$ - r
      dd1 = d1$ - d1
      dd2 = d2$ - d2
      dd3 = d3$ - d3
      dd4 = d4$ - d4
c
      dCL = 0.
      dCY = 0.
      dCr = 0.
      dCm = 0.
      dCn = 0.
      dCDff = 0.
      de = 0.

      dCL = dCL + 0.5*(CLa$+CLa)*da
      dCY = dCY + 0.5*(CYa$+CYa)*da
      dCr = dCr + 0.5*(Cra$+Cra)*da
      dCm = dCm + 0.5*(Cma$+Cma)*da
      dCn = dCn + 0.5*(Cna$+Cna)*da
                
      dCL = dCL + 0.5*(CLb$+CLb)*db
      dCY = dCY + 0.5*(CYb$+CYb)*db
      dCr = dCr + 0.5*(Crb$+Crb)*db
      dCm = dCm + 0.5*(Cmb$+Cmb)*db
      dCn = dCn + 0.5*(Cnb$+Cnb)*db
                
      dCL = dCL + 0.5*(CLp$+CLp)*dp
      dCY = dCY + 0.5*(CYp$+CYp)*dp
      dCr = dCr + 0.5*(Crp$+Crp)*dp
      dCm = dCm + 0.5*(Cmp$+Cmp)*dp
      dCn = dCn + 0.5*(Cnp$+Cnp)*dp
                
      dCL = dCL + 0.5*(CLq$+CLq)*dq
      dCY = dCY + 0.5*(CYq$+CYq)*dq
      dCr = dCr + 0.5*(Crq$+Crq)*dq
      dCm = dCm + 0.5*(Cmq$+Cmq)*dq
      dCn = dCn + 0.5*(Cnq$+Cnq)*dq
                
      dCL = dCL + 0.5*(CLr$+CLr)*dr
      dCY = dCY + 0.5*(CYr$+CYr)*dr
      dCr = dCr + 0.5*(Crr$+Crr)*dr
      dCm = dCm + 0.5*(Cmr$+Cmr)*dr
      dCn = dCn + 0.5*(Cnr$+Cnr)*dr
                     
      dCL = dCL + 0.5*(CLd1$+CLd1)*dd1
      dCY = dCY + 0.5*(CYd1$+CYd1)*dd1
      dCr = dCr + 0.5*(Crd1$+Crd1)*dd1
      dCm = dCm + 0.5*(Cmd1$+Cmd1)*dd1
      dCn = dCn + 0.5*(Cnd1$+Cnd1)*dd1
                        
      dCL = dCL + 0.5*(CLd2$+CLd2)*dd2
      dCY = dCY + 0.5*(CYd2$+CYd2)*dd2
      dCr = dCr + 0.5*(Crd2$+Crd2)*dd2
      dCm = dCm + 0.5*(Cmd2$+Cmd2)*dd2
      dCn = dCn + 0.5*(Cnd2$+Cnd2)*dd2
                        
      dCL = dCL + 0.5*(CLd3$+CLd3)*dd3
      dCY = dCY + 0.5*(CYd3$+CYd3)*dd3
      dCr = dCr + 0.5*(Crd3$+Crd3)*dd3
      dCm = dCm + 0.5*(Cmd3$+Cmd3)*dd3
      dCn = dCn + 0.5*(Cnd3$+Cnd3)*dd3
                        
      dCL = dCL + 0.5*(CLd4$+CLd4)*dd4
      dCY = dCY + 0.5*(CYd4$+CYd4)*dd4
      dCr = dCr + 0.5*(Crd4$+Crd4)*dd4
      dCm = dCm + 0.5*(Cmd4$+Cmd4)*dd4
      dCn = dCn + 0.5*(Cnd4$+Cnd4)*dd4

      dCDff = dCDff + 0.5*(CDffd1$+CDffd1)*dd1
      dCDff = dCDff + 0.5*(CDffd2$+CDffd2)*dd2
      dCDff = dCDff + 0.5*(CDffd3$+CDffd3)*dd3
      dCDff = dCDff + 0.5*(CDffd4$+CDffd4)*dd4

      de = de + 0.5*(ed1$+ed1)*dd1
      de = de + 0.5*(ed2$+ed2)*dd2
      de = de + 0.5*(ed3$+ed3)*dd3
      de = de + 0.5*(ed4$+ed4)*dd4


      write(*,*)
      write(*,*) 'CL =', CL, CL$
      write(*,*) 'CY =', CY, CY$
      write(*,*) 'Cl =', Cr, Cr$
      write(*,*) 'Cm =', Cm, Cm$
      write(*,*) 'Cn =', Cn, Cn$

      write(*,*)
      write(*,*) 'da  = ',da
      write(*,*) 'db  = ',db 
      write(*,*) 'dp  = ',dp 
      write(*,*) 'dq  = ',dq 
      write(*,*) 'dr  = ',dr 
      write(*,*) 'dd1 = ',dd1
      write(*,*) 'dd2 = ',dd2
      write(*,*) 'dd3 = ',dd3
      write(*,*) 'dd4 = ',dd4
c

 2500 format(1x,5(4x,a,4x))
 2510 format(1x,5f12.8)

      write(*,*)
      write(*,2500) ' dCL', ' dCY'
      write(*,2510) dCL, dCY
      write(*,2510) CL$-CL, CY$-CY


      write(*,*)
      write(*,2500) ' dCl',' dCm ',' dCn'
      write(*,2510) dCr, dCm, dCn
      write(*,2510) Cr$-Cr, Cm$-Cm, Cn$-Cn


      write(*,*)
      write(*,2500) 'dCDf',' de '
      write(*,2510) dCDff, de
      write(*,2510) CDff$-CDff, e$-e

C
      STOP
      END



      SUBROUTINE GETVAL(LINE,LKEY,VAL)
      CHARACTER*(*) LINE, LKEY
C
      K = INDEX(LINE,LKEY)
C
      IF(K.GT.0) THEN
        LL = LEN(LINE)
C
        K1 = K + LEN(LKEY)
        K2 =     LEN(LINE)
        IF(K2.LT.K1) RETURN
C
        READ(LINE(K1:K2),*,ERR=10) VAL
      ENDIF
      RETURN
C
 10   WRITE(*,*) 'Read error:', LINE(K1:K2)
      RETURN
      END


      SUBROUTINE STRIP(STRING,NS)
      CHARACTER*(*) STRING
C-------------------------------------------
C     Strips leading blanks off string
C     and returns length of non-blank part.
C-------------------------------------------
      N = LEN(STRING)
C
C---- find last non-blank character
      DO 10 K2=N, 1, -1
        IF(STRING(K2:K2).NE.' ') GO TO 11
   10 CONTINUE
      K2 = 0
   11 CONTINUE
C
C---- find first non-blank character
      DO 20 K1=1, K2
        IF(STRING(K1:K1).NE.' ') GO TO 21
   20 CONTINUE
   21 CONTINUE
C
C---- number of non-blank characters
      NS = K2 - K1 + 1
      IF(NS.EQ.0) RETURN
C
C---- shift STRING so first character is non-blank
      STRING(1:NS) = STRING(K1:K2)
C
C---- pad tail of STRING with blanks
      DO 30 K=NS+1, N
        STRING(K:K) = ' '
   30 CONTINUE
C
      RETURN
      END
