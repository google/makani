
      SUBROUTINE FRPLOT(N,S,X,HK,TH,UE,VE,XTR,FREF,
     &                  XOFF,XSF, YOFF,YSF, CHF)
      DIMENSION S(N+1), X(N+1), HK(N+1), TH(N+1), UE(N+1), VE(N+1)
C------------------------------------------------------------
C     Plots the amplitude A(x) for a specified number 
C     of frequencies.  The frequency values which are 
C     used are set internally in the amplitude calculation
C     routine NTCALC, and displayed here.
C
C     N       number of laminar streamwise points i 
C              (transition is in interval N...N+1)
C     S(i)    streamwise arc length for integrating -a_i = d[ln(A)]/ds
C     X(i)    plotting x coordinate 
C     HK(i)   kinematic shape parameter
C     TH(i)   momentum thickness
C     UE(i)   edge velocity
C     VE(i)   edge kinematic viscosity
C     XTR     transition x location,  should be  X(N) < XTR < X(N+1)
C
C     FREF    reference radian frequency (w/FREF is displayed)
C
C     XOFF    plotting offsets, scales...  Xplot = (X-XOFF)*XSF
C     YOFF                                 Yplot = (Y-YOFF)*YSF
C     XSF
C     YSF
C
C     CHF     character height
C------------------------------------------------------------     
C
C---- max number of streamwise points and frequencies
      PARAMETER (IDIM=300,NFX=15)
ccc      PARAMETER (IDIM=300,NFX=50)
C
      DIMENSION FREQ(NFX), ANF(IDIM,NFX)
C
      IF(N+1 .GT. IDIM) STOP 'FRPLOT: Array overflow.  Increase IDIM.'
C
C---- set number of frequencies plotted
      NFR = NFX
C
C---- calculate wave amplitudes for each frequency
      CALL NTCALC(IDIM,N+1, S,HK,TH,UE,VE,
     &            NFR,FREQ,ANF)
C
C---- plot amplitudes for all frequencies
      X1 = X(N)
      X2 = XTR
      FRAC = (X2-X1)/(X(N+1)-X1)
      DO 10 IFR=1, NFR
C
C------ plot A(x) up to the transition interval
        CALL XYLINE(N,X,ANF(1,IFR),XOFF,XSF,YOFF,YSF,1)
C
C------ plot last bit to the transition location in the transition interval
        Y1 = ANF(N,IFR)
        Y2 = ANF(N,IFR) + FRAC*(ANF(N+1,IFR)-ANF(N,IFR))
        CALL PLOT((X1-XOFF)*XSF,(Y1-YOFF)*YSF,3)
        CALL PLOT((X2-XOFF)*XSF,(Y2-YOFF)*YSF,2)
C
C------ label the curve with its frequency if it grew to more than ANFMIN
        ANFMIN = 0.5
        IF(MAX(ANF(N,IFR),ANF(N+1,IFR)) .GT. ANFMIN) THEN
         XNUM = (X2-XOFF)*XSF + 0.5*CHF
         YNUM = (Y2-YOFF)*YSF - 0.5*CHF
         CALL PLNUMB(XNUM,YNUM,CHF,FREQ(IFR)/FREF,0.0,2)
        ENDIF
C
 10   CONTINUE
C
      RETURN
      END
