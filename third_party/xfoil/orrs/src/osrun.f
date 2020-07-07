
      PROGRAM OSRUN
C---------------------------------------------------------------------------
C     Program for executing and displaying Orr-Sommerfeld solution
C     Usage:
C
C      % osrun [ inputfile ]
C
C     The inputfile contains mean-flow profile data:
C
C        y1 U1 S1
C        y2 U2 S1
C        .  .  .
C        .  .  .
C
C     where S = dU/dy .
C     If the optional argument "inputfile" is missing, then
C     a Falkner-Skan profile will be generated, for a specified  
C     H  or  betaU = x/ue due/dx
C
C     In either case, the user is prompted for R_theta and w_r values.
C
C---------------------------------------------------------------------------
C
      PARAMETER (NMAX=2001)
      DIMENSION ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      DIMENSION UTR(NMAX), UTI(NMAX), UT(NMAX),
     &          VTR(NMAX), VTI(NMAX), VT(NMAX),
     &          WTR(NMAX), WTI(NMAX), WT(NMAX),
     &          CTR(NMAX), CTI(NMAX), CT(NMAX),
     &          PTR(NMAX), PTI(NMAX), PT(NMAX)
      DIMENSION UU(NMAX), VV(NMAX), UV(NMAX), QQ(NMAX)
      DIMENSION TVT(NMAX), TV(NMAX)
      CHARACTER*1 ANS
      CHARACTER*80 FNAME, ARGP1
      DIMENSION XLIN(2), YLIN(2)
C
      DIMENSION AINPUT(10)
      LOGICAL ERROR
C
      DATA CV1 / 7.1 /
C
      LST = 1
      LRE = 1
C
      IDEV = 1
      IDEVRP = 2
      IPSLU = 0
C
      SIZE = 6.0
      PAR = 0.75
C
      CALL PLINITIALIZE
C
C
      N = 1001
      ETAE = 16.0
      GEO = 1.01
C
      CH = 0.021
C
      IF(N.GT.NMAX) STOP 'TEST: Array overflow.'
C
      CALL GETARG0(1,ARGP1)
      IF(ARGP1(1:1).EQ.' ') GO TO 50
C
      FNAME = ARGP1
C
C---- try formatted read first
      OPEN(1,FILE=FNAME,STATUS='OLD',ERR=50)
      DO I=1, NMAX
        READ(1,*,ERR=30,END=25) ETA(I), U(I), S(I)
      ENDDO
C
 25   CLOSE(1)
      N = I-1
      GO TO 80
C
C---- now try unformatted read
 30   CONTINUE
      OPEN(19,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED',ERR=50)
      DO I=1, NMAX
        READ(19,ERR=50,END=35) ETA(I), U(I), S(I)
      ENDDO
C
 35   CLOSE(19)
      N = I-1
      GO TO 80
C
C------------------------------------------------------------------------
C---- no argument specified or read error... get Falkner-Skan parameter
 50   CONTINUE
      WRITE(*,*) 'Enter Falkner-Skan parameter Beta (or H)'
      READ (*,*) PARM
C
      IF(PARM .GT. 1.0) THEN
       write(*,*) 'Enter max y/theta, GEO'
       read (*,*) etae, geo
       H = PARM
       CALL FS(3,2,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C
      ELSE
       write(*,*) 'Enter max y/theta, GEO'
       read (*,*) etae, geo
       BU = PARM / (2.0 - PARM)
       CALL FS(3,1,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C
      ENDIF
C
      WRITE(*,*) 'BetaU, H =', BU, H
ccc      GO TO 90
C
C------------------------------------------------------------------------
C---- normalize input profiles
 80   CONTINUE
      DS = 0.
      TH = 0.
      DO I = 1, N-1
        DY = ETA(I+1) - ETA(I)
        UA = (U(I+1) + U(I))*0.5 / U(N)
        DS = DS + (1.0 - UA)   *DY
        TH = TH + (1.0 - UA)*UA*DY
      ENDDO
C
      H = DS/TH
      DO I = 1, N
        ETA(I) = ETA(I) / TH
        S(I) = S(I) * TH
      ENDDO
      ETAE = ETA(N)
C
      WRITE(*,*) 'H =', H
C
C---------------------
C
 90   CONTINUE


      do i = 1, n
        write(*,'(1x,i5,3g14.6)') i, eta(i), u(i), s(i)
      enddo


      EWT = 1.0/ETAE
C
      DETA = 1.0
      IF(ETAE .GT. 16.01) DETA = 2.0
      IF(ETAE .GT. 30.01) DETA = 5.0
      ETAE = DETA * AINT( ETAE/DETA + 0.51 )
C
      CALL PLOPEN(0.7,IPSLU,IDEV)
      CALL PLOTABS(0.5,1.0,-3)
C
      CALL NEWFACTOR(SIZE)
      CALL GETCOLOR(ICOL0)
C
      CALL PLOT(0.5*PAR,0.0,-3)
C
      CALL NEWPEN(1)
      CALL XAXIS(0.0,0.0,-PAR, 0.2*PAR,0.0, 0.2,0.7*CH,1)
      CALL YAXIS(0.0,0.0,-1.0,DETA*EWT,0.0,DETA,0.7*CH,1)
C
      CALL NEWPEN(2)
      XL = -4.0*CH
      YL = (ETAE-1.5*DETA)*EWT - 0.5*CH
      CALL PLCHAR(XL,YL,CH,'y/ ',0.0,3)
      CALL PLMATH(XL,YL,CH,'  q',0.0,3)
C
C
      UWT = PAR
      CALL NEWPEN(4)
      CALL XYLINE(N,U,ETA,0.0,UWT,0.0,EWT,1)
      CALL NEWPEN(3)
      CALL XYSYMB(N,U,ETA,0.0,UWT,0.0,EWT,0.125*CH,1)
C
      CALL PLFLUSH
C
      RE = 100.0
      OMEGAR = 0.1
      ALPHAR = 2.0*OMEGAR
      ALPHAI = 0.
C
      DO 100 IPASS=1, 50
        ITMAX = 20
C
 95     AINPUT(1) = RE
        AINPUT(2) = OMEGAR
        WRITE(*,2100) AINPUT(1), AINPUT(2)
 2100   FORMAT(1X,' Enter Rtheta, Wreal:', F9.1, F10.5)
        CALL READR(2,AINPUT,ERROR)
        IF(ERROR) GO TO 95
C
        RE     = AINPUT(1)
        OMEGAR = AINPUT(2)
C
        IF(RE .EQ. 0.0) GO TO 101
C
c        RD = RE*H
c        WR = OMEGAR/RE
c        WRITE(*,*) ' '
c        WRITE(*,*) 'Rd* =', RD, '    Wr/Rth =', WR
C
        OMEGAI = 0.0
C
 97     AINPUT(1) = ALPHAR
        AINPUT(2) = ALPHAI
        WRITE(*,2200) AINPUT(1), AINPUT(2)
 2200   FORMAT(1X,' Enter initial ar, ai:', 2F10.5)
        CALL READR(2,AINPUT,ERROR)
        IF(ERROR) GO TO 97
C
        ALPHAR = AINPUT(1)
        ALPHAI = AINPUT(2)
C
        ITLIM = ITMAX
        CALL ORRS(LST,LRE,N,ETA,U,S, RE, ITLIM,
     &            ALPHAR,ALPHAI, OMEGAR,OMEGAI,
     &            UTR,UTI,VTR,VTI,WTR,WTI,CTR,CTI, DELMAX)
C
        CALL OSPRES(N,ETA,U, ALPHAR,ALPHAI, VTR,VTI, PTR,PTI )
C
        DO I=1, N
          UT(I) = SQRT(UTR(I)**2 + UTI(I)**2)
          VT(I) = SQRT(VTR(I)**2 + VTI(I)**2)
          PT(I) = SQRT(PTR(I)**2 + PTI(I)**2)
          UU(I) = 0.5*(UTR(I)*UTR(I) + UTI(I)*UTI(I))
          VV(I) = 0.5*(VTR(I)*VTR(I) + VTI(I)*VTI(I))
          UV(I) = 0.5*(UTR(I)*VTR(I) + UTI(I)*VTI(I))
          QQ(I) = UU(I) + VV(I)
          SLIM = MAX( S(I) , 1.0E-5 )
          TVT(I) = ABS( -UV(I) / SLIM )
C
          TV(I) = SQRT( SQRT( TVT(I)*CV1**3/RE**3 ) )
          IF(I .GE. 3) TV(I) = TV(I-1)
C
          IF(TV(I) .GT. 0.0) THEN
           DO ITER = 1, 10
             CHI = RE*TV(I)
             CHI3 = CHI**3
             CHI3_N = 3.0*CHI**2 * RE
             RES = TVT(I)*(CHI3 + CV1**3) - TV(I)*CHI3
             RES_N = TVT(I)*CHI3_N - TV(I)*CHI3_N - CHI3
             DN = -RES/RES_N
c             write(*,'(1x,2g13.6,2e12.4)') tvt(i), tv(i), res, dn
             TV(I) = TV(I) + DN
c             pause
           ENDDO
          ENDIF
C
        ENDDO
C
        QTHIK = 0.
        DQTDX = 0.
        UPRES = 0.
        TWORK = 0.
        DISS1 = 0.
        DISS2 = 0.
        DISS3 = 0.
        PQINT = 0.
        DO I = 2, N
          UA =  (U(I) +   U(I-1))*0.5
          DU =   U(I) -   U(I-1)
          DY = ETA(I) - ETA(I-1)
C
          URA = (UTR(I) + UTR(I-1))*0.5
          UIA = (UTI(I) + UTI(I-1))*0.5
          VRA = (VTR(I) + VTR(I-1))*0.5
          VIA = (VTI(I) + VTI(I-1))*0.5
          WRA = (WTR(I) + WTR(I-1))*0.5
          WIA = (WTI(I) + WTI(I-1))*0.5
          PRA = (PTR(I) + PTR(I-1))*0.5
          PIA = (PTI(I) + PTI(I-1))*0.5
C
          DUR =  UTR(I) - UTR(I-1)
          DUI =  UTI(I) - UTI(I-1)
          DVR =  VTR(I) - VTR(I-1)
          DVI =  VTI(I) - VTI(I-1)
          DWR =  WTR(I) - WTR(I-1)
          DWI =  WTI(I) - WTI(I-1)
C
          QTHIK = QTHIK + 0.25*(UU(I)+UU(I-1)
     &                         +VV(I)+VV(I-1))*UA*DY
C
          UDUDX = - (ALPHAI*URA + ALPHAR*UIA)*URA
     &            + (ALPHAR*URA - ALPHAI*UIA)*UIA
          VDVDX = - (ALPHAI*VRA + ALPHAR*VIA)*VRA
     &            + (ALPHAR*VRA - ALPHAI*VIA)*VIA
C
          PDUDX = - (ALPHAI*URA + ALPHAR*UIA)*PRA
     &            + (ALPHAR*URA - ALPHAI*UIA)*PIA
          UDPDX = - (ALPHAI*PRA + ALPHAR*PIA)*URA
     &            + (ALPHAR*PRA - ALPHAI*PIA)*UIA
C
          DQTDX = DQTDX + 0.5*(UDUDX + VDVDX)*DY * UA
C
          UPRES = UPRES - 0.5*(UDPDX + PDUDX)*DY
C
          TWORK = TWORK - 0.50*(UV(I)+UV(I-1))*DU
C
          DISS1 = DISS1 + (  ALPHAI*URA + ALPHAR*UIA )**2 * DY
     &                  + (  ALPHAR*URA - ALPHAI*UIA )**2 * DY
C
          DISS2 = DISS2 + (  DVR**2 + DVI**2 ) / DY
C
          DISS3 = DISS3
     &          + 0.5 * ( DUR/DY - ALPHAI*VRA - ALPHAR*VIA )**2 * DY
     &          + 0.5 * ( DUI/DY + ALPHAR*VRA - ALPHAI*VIA )**2 * DY
C
          PQINT = PQINT
     &          - 0.5*URA*DWR - 0.5*(ALPHAI*WRA + ALPHAR*WIA)*VRA * DY
     &          - 0.5*UIA*DWI + 0.5*(ALPHAR*WRA - ALPHAI*WIA)*VIA * DY
        ENDDO
C
        DISS1 = DISS1 / RE
        DISS2 = DISS2 / RE
        DISS3 = DISS3 / RE
        PQINT = PQINT / RE
C
C
        DQTDX = DQTDX / QTHIK
        UPRES = UPRES / QTHIK
        TWORK = TWORK / QTHIK
        DISS1 = DISS1 / QTHIK
        DISS2 = DISS2 / QTHIK
        DISS3 = DISS3 / QTHIK
        PQINT = PQINT / QTHIK
C
        DISS = DISS1 + DISS2 + DISS3
C
        WRITE(*,*)
        WRITE(*,*) 'dEdx, P+Dx+D  :',DQTDX,TWORK+UPRES+PQINT
        WRITE(*,*) 'P Dx D e:', TWORK, UPRES, PQINT, -DISS
        WRITE(*,*)
C
        IF(IPASS.EQ.1) THEN
          CALL SCALIT(N,UT,0.0,USF,ANN,NANN)
          UWT = PAR*USF
C
          CALL SCALIT(N,VT,0.0,VSF,ANN,NANN)
          VWT = PAR*VSF
C
          CALL SCALIT(N,PT,0.0,PSF,ANN,NANN)
          PWT = PAR*PSF
C
          CALL SCALIT(N,QQ,0.0,TSF,ANN,NANN)
          TWT = PAR*TSF
C
          EOFF = 0.
          UOFF = 0.
          POFF = 0.
          TOFF = 0.
C
          PWT = UWT
        ENDIF
C
C
        CALL NEWPEN(3)
C
        XL = PAR + 5.0*CH
C
        YL = ETAE*EWT
        CALL PLCHAR(XL       ,YL       ,    CH,'H     = ',0.0, 8)
        CALL PLNUMB(XL+8.0*CH,YL       ,    CH, H        ,0.0, 3)
C
        YL = YL - 3.5*CH
        CALL PLCHAR(XL       ,YL       ,    CH,'Re    = ',0.0, 8)
        CALL PLMATH(XL+1.9*CH,YL-0.4*CH,0.8*CH,  'q'     ,0.0, 1)
        CALL PLNUMB(XL+8.0*CH,YL       ,    CH, RE       ,0.0,-1)
C
        YL = YL - 2.5*CH
        CALL PLMATH(XL       ,YL       ,    CH,'w q/  = ',0.0, 8)
        CALL PLCHAR(XL       ,YL       ,    CH,'    U   ',0.0, 8)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'r'      ,0.0, 1)
        CALL PLNUMB(XL+8.0*CH,YL       ,    CH, OMEGAR   ,0.0, 5)
C
        YL = YL - 3.5*CH
        CALL PLMATH(XL       ,YL       ,    CH,'a q   = ',0.0, 8)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'r'      ,0.0, 1)
        CALL PLNUMB(XL+8.0*CH,YL       ,    CH, ALPHAR   ,0.0, 5)
C
        YL = YL - 2.5*CH
        CALL PLMATH(XL       ,YL       ,    CH,'a q   = ',0.0, 8)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'i'      ,0.0, 1)
        CALL PLNUMB(XL+8.0*CH,YL       ,    CH, ALPHAI   ,0.0, 5)
C
C
C
        XLIN(1) = -7.5*CH
        XLIN(2) = -1.5*CH
        YLIN(1) =  0.5*CH
        YLIN(2) =  0.5*CH
C
        CALL NEWPEN(2)
C
        XL = PAR + 12.0*CH
        YL = 0.50*ETAE*EWT
C
        CALL NEWCOLORNAME('red')
        CALL XYLINE(N,UTR,ETA,UOFF,UWT,EOFF,EWT,2)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,2)
        CALL PLCHAR(XL       ,YL       ,    CH,'u /U',0.0,4)
        CALL PLMATH(XL       ,YL       ,    CH,' `  ',0.0,4)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'r'  ,0.0,1)
C
        YL = YL - 2.5*CH
C
        CALL NEWCOLORNAME('orange')
        CALL XYLINE(N,UTI,ETA,UOFF,UWT,EOFF,EWT,3)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,3)
        CALL PLCHAR(XL       ,YL       ,    CH,'u /U',0.0,4)
        CALL PLMATH(XL       ,YL       ,    CH,' `  ',0.0,4)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'i'  ,0.0,1)
C
        YL = YL - 2.5*CH
C
        CALL NEWCOLORNAME('yellow')
        CALL XYLINE(N,UT ,ETA,UOFF,UWT,EOFF,EWT,1)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,1)
        CALL PLCHAR(XL       ,YL       ,    CH,'u /U',0.0,4)
        CALL PLMATH(XL       ,YL       ,    CH,' `  ',0.0,4)
        CALL PLCHAR(XL-0.6*CH,YL       ,0.9*CH,'| |' ,0.0,3)
C
C
        YL = YL - 3.5*CH
C
        CALL NEWCOLORNAME('violet')
        CALL XYLINE(N,PTR,ETA,POFF,PWT,EOFF,EWT,5)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,5)
        CALL PLCHAR(XL       ,YL       ,    CH,'p / U ',0.0,6)
        CALL PLMATH(XL       ,YL       ,    CH,' ` r 2',0.0,6)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'r'  ,0.0,1)
C
        YL = YL - 2.5*CH
C
        CALL NEWCOLORNAME('blue')
        CALL XYLINE(N,PTI,ETA,POFF,PWT,EOFF,EWT,6)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,6)
        CALL PLCHAR(XL       ,YL       ,    CH,'p / U ',0.0,6)
        CALL PLMATH(XL       ,YL       ,    CH,' ` r 2',0.0,6)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'i'  ,0.0,1)
C
        YL = YL - 2.5*CH
C
        CALL NEWCOLORNAME('cyan')
        CALL XYLINE(N,PT ,ETA,POFF,PWT,EOFF,EWT,1)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,1)
        CALL PLCHAR(XL       ,YL       ,    CH,'p / U ',0.0,6)
        CALL PLMATH(XL       ,YL       ,    CH,' ` r 2',0.0,6)
        CALL PLCHAR(XL-0.6*CH,YL       ,0.9*CH,'| |'   ,0.0,3)
C
        YL = YL - 3.5*CH
C
        CALL NEWCOLORNAME('green')
        CALL XYLINE(N,QQ ,ETA,TOFF,TWT,EOFF,EWT,2)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,2)
        CALL PLMATH(XL       ,YL+0.2*CH,    CH,' ___ '   ,0.0,5)
        CALL PLMATH(XL       ,YL       ,    CH,'  ` `  2',0.0,8)
        CALL PLCHAR(XL       ,YL       ,    CH,' q q /U ',0.0,8)
C
        YL = YL - 2.5*CH
C
        CALL NEWCOLORNAME('green')
        CALL XYLINE(N,UV ,ETA,TOFF,-10.0*TWT,EOFF,EWT,1)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,1)
        CALL PLMATH(XL       ,YL+0.2*CH,    CH,' ___        ',0.0,12)
        CALL PLMATH(XL       ,YL       ,    CH,'  ` `  2 #  ',0.0,12)
        CALL PLCHAR(XL       ,YL       ,    CH,'-u v /U   10',0.0,12)
C
        YL = YL - 3.5*CH
C
        CALL NEWCOLORNAME('magenta')
        CALL XYLINE(N,TVT,ETA,TOFF,TWT,EOFF,EWT,1)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,1)
        CALL PLMATH(XL       ,YL+0.2*CH,    CH,'n   2    ',0.0,9)
        CALL PLCHAR(XL       ,YL       ,    CH,'  /U     ',0.0,9)
        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 't'  ,0.0,1)
C
        YL = YL - 2.5*CH
C
        CALL NEWCOLORNAME('magenta')
        CALL XYLINE(N,TV,ETA,TOFF,TWT,EOFF,EWT,2)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,2)
        CALL PLMATH(XL       ,YL+0.2*CH,    CH,'n   2',0.0,5)
        CALL PLMATH(XL       ,YL+0.2*CH,    CH,'~'    ,0.0,1)
        CALL PLCHAR(XL       ,YL       ,    CH,'  /U ',0.0,5)
C
        CALL NEWCOLOR(ICOL0)
C
        CALL PLFLUSH
C
 99     CONTINUE
        WRITE(*,*) 'Zoom, Unzoom, Annotate, Hardcopy, Dump ?'
        READ (*,1000) ANS
 1000   FORMAT(A)
C
        IF    (INDEX('Zz',ANS) .NE. 0) THEN
          CALL USETZOOM(.FALSE.,.TRUE.)
          CALL REPLOT(IDEV)
          GO TO 99
        ELSEIF(INDEX('Uu',ANS) .NE. 0) THEN
          CALL CLRZOOM
          CALL REPLOT(IDEV)
          GO TO 99
        ELSEIF(INDEX('Aa',ANS) .NE. 0) THEN
          CALL ANNOT(CH)
          GO TO 99
        ELSEIF(INDEX('Hh',ANS) .NE. 0) THEN
          CALL REPLOT(IDEVRP)
          GO TO 99
        ELSEIF(INDEX('Dd',ANS) .NE. 0) THEN
          LU = 9
          WRITE(LU,9922) ALPHAR,ALPHAI, OMEGAR,OMEGAI
          DO I = 1, N
 9922        FORMAT(1X, 8E16.7)
            WRITE(LU,9922) 
     &         UTR(I), UTI(I), VTR(I), VTI(I), PTR(I), PTI(I)
          ENDDO
          WRITE(*,*) 'Written to fort.9'
C
        ENDIF
C
  100 CONTINUE
  101 CONTINUE
C
      CALL PLCLOSE
      STOP
      END

