      PROGRAM MAPPL1
C-----------------------------------------------------
C     Plots spatial-stability function contours
C     for one shape parameter H value.
C       alpha_i(w,Re ; H)
C       alpha_r(w,Re ; H)
C-----------------------------------------------------
      PARAMETER (NMAX=257,NRX=111,NWX=91)
      REAL ETA(NMAX), U(NMAX), S(NMAX)
C
      REAL AR(NRX,NWX), AI(NRX,NWX)
      REAL RTL(NRX), WSL(NWX)
C
      REAL RT(NRX), WS(NWX)
      REAL X(NRX,NWX), Y(NRX,NWX)
C
      CHARACTER*1 ANS
      CHARACTER*80 FNAME, ARGP1, ARGP2
      LOGICAL LABCON, YES, MANUAL
C
C---- Plotting flag
      IDEV = 1   ! X11 window only
c     IDEV = 2   ! B&W PostScript output file only (no color)
c     IDEV = 3   ! both X11 and B&W PostScript file
c     IDEV = 4   ! Color PostScript output file only 
c     IDEV = 5   ! both X11 and Color PostScript file 
C
      IDEV = 3
      IPSLU = 0
C
      SIZE = 4.0
      CH = 0.020
      CHL = 0.018
      CHX = CH*1.7
C
      CALL PLINITIALIZE
C
C
      CALL GETARG0(1,ARGP1)
      CALL GETARG0(2,ARGP2)

      IF(ARGP1 .EQ. ' ') THEN
       WRITE(*,*) 'Usage:  % mappl1  OSmapfile  [ cntrparfile ]'
       STOP
      ENDIF
C
C---- set expeced format of source files
      IFORM = -1 !  unknown
ccc   IFORM = 0  !  binary
ccc   IFORM = 1  !  ascii

      CALL READOS1(ARGP1,IFORM,
     &            N,NMAX,ETA,U,S,
     &            NRP,NWP,
     &            RTL,WSL,HH, AR,AI,
     &            NRX,NWX)
 5    CONTINUE
C
      NR = NRP
      NW = NWP
C
      DO IR=1, NR
        RT(IR) = 10.0 ** RTL(IR)
      ENDDO
C
      DO IW=1, NW
        WS(IW) = 10.0 ** WSL(IW)
      ENDDO
C
      RTLMIN = RTL(1 )
      RTLMAX = RTL(NR)
C
      WRLMIN = WSL(1 ) - 0.5*RTL(NR)
      WRLMAX = WSL(NW) - 0.5*RTL(1 )
C
      ARMIN = AR(1,1)
      ARMAX = AR(1,1)
      AIMIN = AI(1,1)
      AIMAX = AI(1,1)
      DO IW=1, NW
        DO IR=1, NR
          ARMIN = MIN(ARMIN,AR(IR,IW))
          ARMAX = MAX(ARMAX,AR(IR,IW))
          AIMIN = MIN(AIMIN,AI(IR,IW))
          AIMAX = MAX(AIMAX,AI(IR,IW))
        ENDDO
      ENDDO
C
C
C---- log-log Rtheta-W plot exponent limits
C      I1 = INT(RTLMIN+100.001) - 100
C      I2 = INT(RTLMAX+100.999) - 100
C      J1 = INT(WRLMIN+100.001) - 100
C      J2 = INT(WRLMAX+100.999) - 100
C
      I1 = 0
      I2 = 6
      J1 = -6
      J2 = 1
C
      RTLMIN = FLOAT(I1)
      RTLMAX = FLOAT(I2)
      WRLMIN = FLOAT(J1)
      WRLMAX = FLOAT(J2)
C
CCC      SF = AMIN1( 1.0/(RTLMAX-RTLMIN) , 1.0/(WRLMAX-WRLMIN) )
      SF = 1.0/(RTLMAX-RTLMIN)
C
      DO IW=1, NW
        DO IR=1, NR
          WRL = WSL(IW) - 0.5*RTL(IR)
          X(IR,IW) = (RTL(IR)-RTLMIN) * SF
          Y(IR,IW) = (WRL       -WRLMIN) * SF
        ENDDO
      ENDDO
C
      FNAME = ARGP2
      IF(FNAME.EQ.' ') THEN
       CALL ASKS('Enter contour parameter filename (or return)^',FNAME)
      ENDIF
      MANUAL = FNAME .EQ. ' '
C
      CALL PLOPEN(0,IPSLU,IDEV)
      CALL NEWFACTOR(SIZE)
      CALL PLOT(12.0*CH,8.0*CH,-3)
C
      DO 9000 IPASS=1, 2
C
      DO 50 I=I1, I2
        XLIN  = (FLOAT(I) -RTLMIN) * SF
        YLIN1 = (FLOAT(J1)-WRLMIN) * SF
        YLIN2 = (FLOAT(J2)-WRLMIN) * SF
        CALL NEWPEN(1)
        CALL PLOT(XLIN,YLIN1,3)
        CALL PLOT(XLIN,YLIN2,2)
C
        CALL NEWPEN(2)
        RI = FLOAT(I)
        CALL PLCHAR(XLIN-1.0*CH,YLIN1-2.5*CH,1.2*CH,'10',0.0, 2)
        CALL PLNUMB(XLIN+1.4*CH,YLIN1-1.8*CH,0.9*CH,RI  ,0.0,-1)
   50 CONTINUE
C
      DO 55 J=J1, J2
        YLIN  = (FLOAT(J) -WRLMIN) * SF
        XLIN1 = (FLOAT(I1)-RTLMIN) * SF
        XLIN2 = (FLOAT(I2)-RTLMIN) * SF
        CALL NEWPEN(1)
        CALL PLOT(XLIN1,YLIN,3)
        CALL PLOT(XLIN2,YLIN,2)
C
        CALL NEWPEN(2)
        RJ = FLOAT(J)
        CALL PLCHAR(XLIN1-4.4*CH,YLIN-0.6*CH,1.2*CH,'10',0.0, 2)
        CALL PLNUMB(XLIN1-2.0*CH,YLIN-0.0*CH,0.9*CH,RJ  ,0.0,-1)
   55 CONTINUE
C
      CALL NEWPEN(3)
      XLAB = (FLOAT((I1+I2)/2) + 0.5 - RTLMIN) * SF - 1.0*CH
      YLAB = (FLOAT( J1      )       - WRLMIN) * SF - 3.7*CH
      CALL PLCHAR(XLAB       ,YLAB       ,1.7*CH,'R',0.0,1)
      CALL PLCHAR(XLAB+1.3*CH,YLAB       ,1.4*CH,'e',0.0,1)
      CALL PLMATH(XLAB+2.6*CH,YLAB-0.6*CH,1.2*CH,'q',0.0,1)
C
      CALL NEWPEN(3)
      XLAB = (FLOAT( I1      )       - RTLMIN) * SF - 8.2*CH
      YLAB = (FLOAT((J1+J2)/2) + 0.5 - WRLMIN) * SF - 0.9*CH
      CALL PLMATH(XLAB,YLAB,1.7*CH,'wq'  ,0.0,2)
      CALL PLCHAR(XLAB,YLAB,1.7*CH,'  /u',0.0,4)
      CALL PLCHAR(XLAB+6.7*CH,YLAB-0.5*CH,1.2*CH,'e',0.0,1)
C
      CALL NEWPEN(3)
      XLAB = 0.5*CH
      YLAB = (FLOAT(J2)-WRLMIN)*SF + 1.0*CH
      CALL PLMATH(XLAB       ,YLAB        ,2.0*CH,'a' ,0.0,1)
      CALL PLMATH(XLAB+2.8*CH,YLAB        ,2.0*CH, 'q',0.0,1)
      IF(IPASS.EQ.1) THEN
       CALL PLCHAR(XLAB+ 1.6*CH,YLAB-0.4*CH,1.2*CH,'i',0.0,1)
      ELSE
       CALL PLCHAR(XLAB+ 1.6*CH,YLAB-0.4*CH,1.2*CH,'r',0.0,1)
      ENDIF
      CALL PLCHAR(XLAB+ 6.5*CH,YLAB,1.6*CH,'contours',0.0,8)
C
      XLAB = (FLOAT(I2)-RTLMIN)*SF - 10.0*1.5*CH
      CALL PLCHAR(XLAB       ,YLAB,1.6*CH,'H = ',0.0,4)
      CALL PLNUMB(XLAB+6.0*CH,YLAB,1.6*CH, HH   ,0.0,3)
C
      IF(IPASS.EQ.1) WRITE(*,*) 'ai limits:', AIMIN, AIMAX
      IF(IPASS.EQ.2) WRITE(*,*) 'ar limits:', ARMIN, ARMAX
C
      IF(.NOT.MANUAL) OPEN(19,FILE=FNAME,STATUS='OLD')
C
  800 CONTINUE
c
cc---- plot function grid
c      call newpen(1)
c      do 60 ir=1, nr
c        call plot(x(ir,1),y(ir,1),3)
c        do 610 iw=2, nw
c          call plot(x(ir,iw),y(ir,iw),2)
c  610   continue
c   60 continue
c      do 70 iw=1, nw
c        call plot(x(1,iw),y(1,iw),3)
c        do 710 ir=2, nr
c          call plot(x(ir,iw),y(ir,iw),2)
c  710   continue
c   70 continue
cc
c
      IF(MANUAL) THEN
       WRITE(*,*) ' '
       CALL ASKR('Enter starting contour level^',ALOW)
       CALL ASKR('Enter contour level increment (+/-)^',DA)
       CALL ASKI('Enter contour line thickness (1-5)^',LPEN)
       CALL ASKL('Add numerical labels to contours ?^',LABCON)
      ELSE
       READ(19,*,END=900) ALOW, DA, LPEN, LABCON
       write(*,*)  ALOW, DA, LPEN, LABCON
       IF(ALOW .EQ. 999.0) GO TO 900
      ENDIF
C
C
C**** plot and label contours
C
      CALL NEWPEN(LPEN)
C
C---- go over contour levels
      DO 80 IA = 0, 12345
C
C------ set contour level
        ACON = ALOW + DA*FLOAT(IA)
C
C
        IF(IPASS.EQ.1) THEN
C------- skip out if outside limits
         IF((DA.GT.0.0 .AND. ACON.GT.AIMAX) .OR. 
     &      (DA.LT.0.0 .AND. ACON.LT.AIMIN)      ) GO TO 81
C
           CALL CONTGRID(NRX,NWX,NR,NW,X,Y,AI,ACON,0.0,0.0,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLAB(NRX,NWX,NR,NW,X,Y,AI,ACON,1.0,1.0,CHL,3,1)
          CALL CONLAB(NRX,NWX,NR,NW,X,Y,AI,ACON,1.0,1.0,CHL,3,2)
          CALL CONLAB(NRX,NWX,NR,NW,X,Y,AI,ACON,1.0,1.0,CHL,3,3)
         ENDIF
        ELSE
C------- skip out if outside limits
         IF((DA.GT.0.0 .AND. ACON.GT.ARMAX) .OR. 
     &      (DA.LT.0.0 .AND. ACON.LT.ARMIN)      ) GO TO 81
C
         CALL CONTGRID(NRX,NWX,NR,NW,X,Y,AR,ACON,
     &    0.0,0.0,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLAB(NRX,NWX,NR,NW,X,Y,AR,ACON,1.0,1.0,CHL,3,1)
          CALL CONLAB(NRX,NWX,NR,NW,X,Y,AR,ACON,1.0,1.0,CHL,3,2)
          CALL CONLAB(NRX,NWX,NR,NW,X,Y,AR,ACON,1.0,1.0,CHL,3,3)
         ENDIF
        ENDIF
   80 CONTINUE
   81 CONTINUE
C
      CALL PLFLUSH
C
      IF(MANUAL) THEN
       CALL ASKL('Add more contours ?^',YES)
       IF(YES) GO TO 800
      ELSE
       GO TO 800
      ENDIF
C
  900 IF(IPASS.LT.2) CALL PLOT((RTLMAX-RTLMIN)*SF+12.0*CH,0.0,-3)
C
 9000 CONTINUE
C
      IF(.NOT.MANUAL) THEN
       CLOSE(19)
       CALL ASKS('Hit <cr>^',ANS)
      ENDIF
C
      CALL PLOT(0.0,0.0,+999)
C
      STOP
      END


