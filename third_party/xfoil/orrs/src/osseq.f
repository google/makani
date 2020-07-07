
      PROGRAM OSSEQ
C--------------------------------------------------------------------
C     Program for executing and displaying Orr-Sommerfeld solution
C--------------------------------------------------------------------
C
      PARAMETER (NMAX=2001)
      DIMENSION ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      DIMENSION UTR(NMAX), UTI(NMAX), UT(NMAX),
     &          VTR(NMAX), VTI(NMAX), VT(NMAX),
     &          WTR(NMAX), WTI(NMAX), WT(NMAX),
     &          CTR(NMAX), CTI(NMAX), CT(NMAX),
     &          PTR(NMAX), PTI(NMAX), PT(NMAX)
      DIMENSION UU(NMAX), VV(NMAX), UV(NMAX), QQ(NMAX)
      CHARACTER*1 ANS
      CHARACTER*80 FNAME, ARGP1
      DIMENSION XLIN(2), YLIN(2)
C
      DIMENSION AINPUT(10)
      LOGICAL ERROR
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
      N = 2001
      ETAE = 16.0
      GEO = 1.01
C
      CH = 0.021
C
      IF(N.GT.NMAX) STOP 'TEST: Array overflow.'
C
      CALL GETARG(1,ARGP1)
      IF(ARGP1(1:1).EQ.' ') GO TO 50
C
      FNAME = ARGP1
C
C---- try formatted read first
      OPEN(1,FILE=FNAME,STATUS='OLD',ERR=50)
      READ(1,*,ERR=30) N, H
      DO I=1, N
        READ(1,*) ETA(I), U(I), S(I)
      ENDDO
      CLOSE(1)
      ETAE = ETA(N)
      GO TO 90
C
C---- now try unformatted read
 30   CONTINUE
      OPEN(19,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED',ERR=50)
      READ(19,ERR=50) N, H
      READ(19) (ETA(I),I=1, N)
      READ(19) (U(I)  ,I=1, N)
      READ(19) (S(I)  ,I=1, N)
      ETAE = ETA(N)
      CLOSE(19)
      GO TO 90
C
C---- no argument specified or read error... get Falkner-Skan parameter
 50   CONTINUE
      WRITE(*,*) 'Enter Falkner-Skan parameter Beta (or H)'
      READ (*,*) BETA
C
      IF(BETA .GT. 1.0) THEN
       write(*,*) 'Enter ETAE, GEO'
       read (*,*) etae, geo
       H = BETA
       CALL FS(3,2,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C
      ELSE
       write(*,*) 'Enter ETAE, GEO'
       read (*,*) etae, geo
       BU = BETA / (2.0 - BETA)
       CALL FS(3,1,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C
      ENDIF
C---------------------
C
 90   CONTINUE
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
      WRITE(*,*) 'Enter Rtheta1, Rtheta2:'
      READ(*,*) RTH1, RTH2
C
      WRITE(*,*) 'Enter wr1, wr2:'
      READ(*,*) OMG1, OMG2
C
      WRITE(*,*) 'Enter number of steps:'
      READ(*,*) NPASS
C
      WRITE(*,*) 'Enter initial ar, ai:'
      READ(*,*) ALPHAR, ALPHAI
C
      OPEN(19,FILE='a.dat',STATUS='unknown')
      REWIND(19)
C
      DO 100 IPASS=1, NPASS
        ITMAX = 20
C
        FRAC = FLOAT(IPASS-1) / FLOAT(NPASS-1)
C
        RE     = RTH1 * EXP( LOG(RTH2/RTH1) * FRAC )
        OMEGAR = OMG1 * EXP( LOG(OMG2/OMG1) * FRAC )
C
        OMEGAI = 0.0
C
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
        WRITE(19,9944) RE, OMEGAR, DQTDX, TWORK, UPRES, PQINT
 9944   FORMAT(1X,8E14.5)
C
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
cC
c        CALL NEWCOLORNAME('yellow')
c        CALL XYLINE(N,UT ,ETA,UOFF,UWT,EOFF,EWT,1)
c        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,1)
c        CALL PLCHAR(XL       ,YL       ,    CH,'u /U',0.0,4)
c        CALL PLMATH(XL       ,YL       ,    CH,' `  ',0.0,4)
c        CALL PLCHAR(XL-0.6*CH,YL       ,0.9*CH,'| |' ,0.0,3)
cC
cC
c        YL = YL - 3.5*CH
cC
c        CALL NEWCOLORNAME('violet')
c        CALL XYLINE(N,PTR,ETA,POFF,PWT,EOFF,EWT,5)
c        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,5)
c        CALL PLCHAR(XL       ,YL       ,    CH,'p / U ',0.0,6)
c        CALL PLMATH(XL       ,YL       ,    CH,' ` r 2',0.0,6)
c        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'r'  ,0.0,1)
cC
c        YL = YL - 2.5*CH
cC
c        CALL NEWCOLORNAME('blue')
c        CALL XYLINE(N,PTI,ETA,POFF,PWT,EOFF,EWT,6)
c        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,6)
c        CALL PLCHAR(XL       ,YL       ,    CH,'p / U ',0.0,6)
c        CALL PLMATH(XL       ,YL       ,    CH,' ` r 2',0.0,6)
c        CALL PLCHAR(XL+0.9*CH,YL-0.3*CH,0.8*CH, 'i'  ,0.0,1)
cC
c        YL = YL - 2.5*CH
cC
        CALL NEWCOLORNAME('cyan')
        CALL XYLINE(N,PT ,ETA,POFF,PWT,EOFF,EWT,1)
        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,1)
        CALL PLCHAR(XL       ,YL       ,    CH,'p / U ',0.0,6)
        CALL PLMATH(XL       ,YL       ,    CH,' ` r 2',0.0,6)
        CALL PLCHAR(XL-0.6*CH,YL       ,0.9*CH,'| |'   ,0.0,3)
C
        YL = YL - 3.5*CH
C
c        CALL NEWCOLORNAME('green')
c        CALL XYLINE(N,QQ ,ETA,TOFF,TWT,EOFF,EWT,2)
c        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,2)
c        CALL PLMATH(XL       ,YL+0.2*CH,    CH,' ___ '   ,0.0,5)
c        CALL PLMATH(XL       ,YL       ,    CH,'  ` `  2',0.0,8)
c        CALL PLCHAR(XL       ,YL       ,    CH,' q q /U ',0.0,8)
cC
c        YL = YL - 2.5*CH
cC
c        CALL NEWCOLORNAME('green')
c        CALL XYLINE(N,UV ,ETA,TOFF,-10.0*TWT,EOFF,EWT,1)
c        CALL XYLINE(2,XLIN,YLIN,-XL,1.0,-YL,1.0,1)
c        CALL PLMATH(XL       ,YL+0.2*CH,    CH,' ___        ',0.0,12)
c        CALL PLMATH(XL       ,YL       ,    CH,'  ` `  2 #  ',0.0,12)
c        CALL PLCHAR(XL       ,YL       ,    CH,'-u v /U   10',0.0,12)
cC
        CALL NEWCOLOR(ICOL0)
C
        CALL PLFLUSH
C
  100 CONTINUE
  101 CONTINUE
C
      CLOSE(19)
C
      CALL PLCLOSE
      STOP
      END



