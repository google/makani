      PROGRAM MAPPL0
C----------------------------------------------------------
C     Plots spatial-stability same-function contours
C     for a range of shape parameters H.
C       alpha_i(w,Re ; H) = aspec
C     Intended for plotting neutral curves, where aspec=0
C----------------------------------------------------------
      PARAMETER (NMAX=301,NRX=111,NWX=91,NHX=25)
      INTEGER N(NHX)
      REAL ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
C
      INTEGER NR(NHX),NW(NHX)
      REAL HH(NHX)

      REAL AR(NRX,NWX,NHX), AI(NRX,NWX,NHX)
      REAL RTL(NRX,NHX), WSL(NWX,NHX)
C
      REAL RT(NRX,NHX), WS(NWX,NHX)
      REAL X(NRX,NWX), Y(NRX,NWX)
C
      REAL ARMIN(NHX),ARMAX(NHX),
     &     AIMIN(NHX),AIMAX(NHX),
     &     WAIMIN(NHX),
     &     RAIMIN(NHX)

      CHARACTER*1 ANS
      CHARACTER*80 FNAME, ARGP1, ARGP2
      CHARACTER*10 CLABEL
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
      CALL GETARG0(1,ARGP1)
      CALL GETARG0(2,ARGP2)

      IF(ARGP1 .EQ. ' ') THEN
       WRITE(*,*) 'Usage:  % mappl1 OSmapfilelist  [ cntrparfile ]'
       STOP
      ENDIF
C
      LU = 1
      OPEN(LU,FILE=ARGP1,STATUS='OLD',ERR=990)
      DO IH = 1, NHX
 2      CONTINUE
        READ(LU,'(A)',END=5) FNAME
        IF(FNAME .EQ. ' ') GO TO 2
        
C------ set expeced format of source files
        IFORM = -1 !  unknown
ccc     IFORM = 0  !  binary
ccc     IFORM = 1  !  ascii
        CALL READOS1(FNAME,IFORM,
     &               N(IH),NMAX,ETA(1,IH),U(1,IH),S(1,IH),
     &               NRP,NWP,
     &               RTL(1,IH),WSL(1,IH),HH(IH), AR(1,1,IH),AI(1,1,IH),
     &               NRX,NWX)
        NH = IH
        NR(IH) = NRP
        NW(IH) = NWP

c        write(*,*)
c        do ir = 1, nrp
c          write(*,*) hh(ih), exp(rtl(ir,ih)), ai(
c        enddo
c        pause
      ENDDO
 5    CONTINUE
C
      DO IH = 1, NH
        DO IR=1, NR(IH)
          RT(IR,IH) = 10.0 ** RTL(IR,IH)
        ENDDO
        DO IW=1, NW(IH)
          WS(IW,IH) = 10.0 ** WSL(IW,IH)
        ENDDO
      ENDDO
C
      IH = 1
      RTLMIN = RTL(1     ,IH)
      RTLMAX = RTL(NR(IH),IH)
      WRLMIN = WSL(1     ,IH) - 0.5*RTL(NR(IH),IH)
      WRLMAX = WSL(NW(IH),IH) - 0.5*RTL(1     ,IH)

      DO IH = 1, NH
        RTLMIN = MIN(RTLMIN,RTL(1     ,IH))
        RTLMAX = MAX(RTLMAX,RTL(NR(IH),IH))
        WRLMIN = MIN(WRLMIN,WSL(1     ,IH) - 0.5*RTL(NR(IH),IH))
        WRLMAX = MAX(WRLMAX,WSL(NW(IH),IH) - 0.5*RTL(1     ,IH))

        ARMIN(IH) = AR(1,1,IH)
        ARMAX(IH) = AR(1,1,IH)
        AIMIN(IH) = AI(1,1,IH)
        AIMAX(IH) = AI(1,1,IH)
        WAIMIN(IH) = WSL(1,IH) - 0.5*RTL(1,IH)
        RAIMIN(IH) = RTL(1,IH)
        DO IW=1, NW(IH)
          DO IR=1, NR(IH)
            IF(AI(IR,IW,IH) .LT. AIMIN(IH)) THEN
             WAIMIN(IH) = WSL(IW,IH) - 0.5*RTL(IR,IH)
             RAIMIN(IH) = RTL(IR,IH)
            ENDIF
            ARMIN(IH) = MIN(ARMIN(IH),AR(IR,IW,IH))
            ARMAX(IH) = MAX(ARMAX(IH),AR(IR,IW,IH))
            AIMIN(IH) = MIN(AIMIN(IH),AI(IR,IW,IH))
            AIMAX(IH) = MAX(AIMAX(IH),AI(IR,IW,IH))
          ENDDO
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
      J2 = 0
C
      RTLMIN = FLOAT(I1)
      RTLMAX = FLOAT(I2)
      WRLMIN = FLOAT(J1)
      WRLMAX = FLOAT(J2)
C
CCC      SF = AMIN1( 1.0/(RTLMAX-RTLMIN) , 1.0/(WRLMAX-WRLMIN) )
      SF = 1.0/(RTLMAX-RTLMIN)
C

      IF(ARGP2.EQ.' ') THEN
       CALL ASKS('Enter contour parameter filename (or return)^',ARGP2)
      ENDIF
      MANUAL = ARGP2 .EQ. ' '
      IF(.NOT.MANUAL) OPEN(19,FILE=ARGP2,STATUS='OLD')

      CALL PLOPEN(0,IPSLU,IDEV)
      CALL NEWFACTOR(SIZE)
      CALL PLOT(12.0*CH,8.0*CH,-3)
C

      DO 100 IH = 1, NH
      REWIND(19)

      DO IW = 1, NW(IH)
        DO IR = 1, NR(IH)
          WRL = WSL(IW,IH) - 0.5*RTL(IR,IH)
          X(IR,IW) = (RTL(IR,IH)-RTLMIN) * SF
          Y(IR,IW) = (WRL       -WRLMIN) * SF
        ENDDO
      ENDDO
C
      DO 90 IPASS=1, 1   ! ai only
ccc   DO 90 IPASS=1, 2   ! ai and ar
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
      CALL PLMATH(XLAB+2.8*CH,YLAB        ,2.0*CH, 'q= ',0.0,3)
      CALL PLCHAR(XLAB+2.8*CH,YLAB        ,2.0*CH, '  0',0.0,3)
      IF(IPASS.EQ.1) THEN
       CALL PLCHAR(XLAB+ 1.6*CH,YLAB-0.4*CH,1.2*CH,'i',0.0,1)
      ELSE
       CALL PLCHAR(XLAB+ 1.6*CH,YLAB-0.4*CH,1.2*CH,'r',0.0,1)
      ENDIF
      CALL PLCHAR(XLAB+ 10.5*CH,YLAB,1.6*CH,'contours',0.0,8)
C
c      XLAB = (FLOAT(I2)-RTLMIN)*SF - 10.0*1.5*CH
c      CALL PLCHAR(XLAB       ,YLAB,1.6*CH,'H = ',0.0,4)
c      CALL PLNUMB(XLAB+6.0*CH,YLAB,1.6*CH, HH   ,0.0,3)
C
c      IF(IPASS.EQ.1) WRITE(*,*) 'ai limits:', AIMIN, AIMAX
c      IF(IPASS.EQ.2) WRITE(*,*) 'ar limits:', ARMIN, ARMAX
C
      IF(.NOT.MANUAL) OPEN(19,FILE=ARGP2,STATUS='OLD')
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
       READ(19,*,END=100) ALOW, DA, LPEN, LABCON
c       write(*,*)  ALOW, DA, LPEN, LABCON
       IF(ALOW .EQ. 999.0) GO TO 100
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
        IF(HH(IH) .LT. 9.995) THEN
cc       WRITE(CLABEL,'(A,F3.1)') 'H = ', HH(IH)
         WRITE(CLABEL,'(F3.1)') HH(IH)
        ELSE
cc       WRITE(CLABEL,'(A,F4.1)') 'H = ', HH(IH)
         WRITE(CLABEL,'(F4.1)') HH(IH)
        ENDIF
C
        IF(IPASS.EQ.1) THEN
C------- skip out if outside limits
         IF((DA.GT.0.0 .AND. ACON.GT.AIMAX(IH)) .OR. 
     &      (DA.LT.0.0 .AND. ACON.LT.AIMIN(IH))      ) GO TO 81
C
         CALL CONTGRID(NRX,NWX,NR(IH),NW(IH),X,Y,
     &         AI(1,1,IH),ACON,0.0,0.0,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLABC(NRX,NWX,NR(IH),NW(IH),X,Y,AI(1,1,IH),
     &         ACON,CLABEL,1.0,1.0,CHL,3,1)
          CALL CONLABC(NRX,NWX,NR(IH),NW(IH),X,Y,AI(1,1,IH),
     &         ACON,CLABEL,1.0,1.0,CHL,3,2)
          CALL CONLABC(NRX,NWX,NR(IH),NW(IH),X,Y,AI(1,1,IH),
     &         ACON,CLABEL,1.0,1.0,CHL,3,3)
         ENDIF
        ELSE
C------- skip out if outside limits
         IF((DA.GT.0.0 .AND. ACON.GT.ARMAX(IH)) .OR. 
     &      (DA.LT.0.0 .AND. ACON.LT.ARMIN(IH))      ) GO TO 81
C
         CALL CONTGRID(NRX,NWX,NR,NW,X,Y,AR,ACON,
     &    0.0,0.0,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLABC(NRX,NWX,NR(IH),NW(IH),X,Y,AR(1,1,IH),
     &         ACON,CLABEL,1.0,1.0,CHL,3,1)
          CALL CONLABC(NRX,NWX,NR(IH),NW(IH),X,Y,AR(1,1,IH),
     &         ACON,CLABEL,1.0,1.0,CHL,3,2)
          CALL CONLABC(NRX,NWX,NR(IH),NW(IH),X,Y,AR(1,1,IH),
     &         ACON,CLABEL,1.0,1.0,CHL,3,3)
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
  90  CONTINUE
 100  CONTINUE

  900 IF(IPASS.LT.2) CALL PLOT((RTLMAX-RTLMIN)*SF+12.0*CH,0.0,-3)
C

      IF(.NOT.MANUAL) THEN
       CLOSE(19)
       CALL ASKS('Hit <cr>^',ANS)
      ENDIF
C
      CALL PLOT(0.0,0.0,+999)

      write(*,*) '#   H     -ai_min     w       Rt'
      DO IH = 1, NH
        write(*,'(1x,f7.3,f10.5,F10.5,F10.2)') 
     &     HH(IH), -AIMIN(IH), 10.0**WAIMIN(IH), 10.0**RAIMIN(IH)
      ENDDO


      STOP

 990  WRITE(*,*) 'File ', ARGP1, ' not found'
      STOP
      END ! MAPPL0


