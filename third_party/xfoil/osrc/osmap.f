      SUBROUTINE OSMAP(RSP,WSP,HSP,
     &                ALFR,
     &                ALFR_R, ALFR_W, ALFR_H,
     &                ALFRW_R,ALFRW_W,ALFRW_H ,
     &                ALFI,
     &                ALFI_R, ALFI_W, ALFI_H,
     &                ALFIW_R,ALFIW_W,ALFIW_H , OK)
C---------------------------------------------------------------------
C    
C     Returns real and imaginary parts of complex wavenumber (Alpha) 
C     eigenvalue from Orr-Sommerfeld spatial-stability solution 
C     with mean profiles characterized by shape parameter H.  
C     Also returns the sensitivities of Alpha with respect to the 
C     input parameters.
C
C     The eigenvalue Alpha(Rtheta,W,H) is stored as a 3-D array at 
C     discrete points, which is then interpolated to any (Rtheta,W,H)
C     via a tricubic spline.  The spline coordinates actually used are:
C
C       RL = log10(Rtheta)    
C       WL = log10(W) + 0.5 log10(Rtheta)
C       HL = H
C
C
C      Input:
C      ------
C        RSP      momentum thickness Reynolds number  Rtheta = Theta Ue / v
C        WSP      normalized disturbance frequency         W = w Theta/Ue
C        HSP      shape parameter of mean profile          H = Dstar/Theta
C
C      Output:
C      -------
C        ALFR     real part of complex wavenumber * Theta
C        ALFR_R   d(ALFR)/dRtheta
C        ALFR_W   d(ALFR)/dW
C        ALFR_H   d(ALFR)/dH
C        ALFRW_R  d(dALFR/dW)/dRtheta
C        ALFRW_W  d(dALFR/dW)/dW
C        ALFRW_H  d(dALFR/dW)/dH
C
C        ALFI     imag part of complex wavenumber * Theta
C        ALFI_R   d(ALFI)/dRtheta
C        ALFI_W   d(ALFI)/dW
C        ALFI_H   d(ALFI)/dH
C        ALFIW_R  d(dALFI/dW)/dRtheta
C        ALFIW_W  d(dALFI/dW)/dW
C        ALFIW_H  d(dALFI/dW)/dH
C
C        OK     T  if look up was successful; all values returned are valid
C               F  if point fell outside (RL,WL) spline domain limits; 
C                  all values (ALFR, ALFR_R, etc.) are returned as zero.
C                  Exception: If points only falls outside HL spline limits,
C                  then the HL limit is used and an ALFR value is calculated,
C                  but OK is still returned as F.
C                  
C---------------------------------------------------------------------
      LOGICAL OK
C
C
      REAL B(2,2), BR(2,2), BW(2,2), BH(2,2),
     &            BRW(2,2),BRH(2,2),BWH(2,2),BRWH(2,2)
      REAL C(2)  , CR(2)  , CW(2)  , CH(2)  ,
     &            CRW(2)  ,CRH(2)  ,CWH(2)  ,CRWH(2)
C
      REAL AINT(2),
     &     AINT_R(2), AINT_W(2), AINT_H(2),
     &     AINTW_R(2),AINTW_W(2),AINTW_H(2)
C
      PARAMETER (NRX=31, NWX=41, NHX=21)
      COMMON /AICOM_I/ NR, NW, NH,
     &               IC1, IC2,
     &               IW1(NHX), IW2(NHX), IR1(NHX),IR2(NHX)
C
C---------------------------------------------------------------
C---- single-precision OS data file
      REAL*4 RLSP, WLSP, HLSP,
     &    RINCR, WINCR, RL, WL, HL,
     &    A, AR, AW, AH, ARW, ARH, AWH, ARWH
C
C---- native-precision OS data file
c      REAL RLSP, WLSP, HLSP,
c     &    RINCR, WINCR, RL, WL, HL,
c     &    A, AR, AW, AH, ARW, ARH, AWH, ARWH
C---------------------------------------------------------------
C
      COMMON /AICOM_R/ RINCR, WINCR, RL(NRX), WL(NWX), HL(NHX),
     &                A(NRX,NWX,NHX,2),
     &               AR(NRX,NWX,NHX,2),
     &               AW(NRX,NWX,NHX,2),
     &               AH(NRX,NWX,NHX,2),
     &              ARW(NRX,NWX,NHX,2),
     &              ARH(NRX,NWX,NHX,2),
     &              AWH(NRX,NWX,NHX,2),
     &             ARWH(NRX,NWX,NHX,2)
C
      LOGICAL LOADED, NOFILE
      SAVE LOADED, NOFILE
C
C---- set OSFILE to match the absolute OS database filename
      CHARACTER*128 OSFILE
      INTEGER LOSF

      DATA OSFILE / '/var/local/codes/orrs/osmap.dat' /
c      DATA OSFILE
c     &/'/afs/athena.mit.edu/course/16/16_d0006/Codes/orrs/osmap_lx.dat'/
C
      DATA LOADED, NOFILE / .FALSE. , .FALSE. /
C
C---- set ln(10) for derivatives of log10 function
      DATA AL10 /2.302585093/
C
C
C---- set default returned variables in case of error, or OS map not available
      ALFR = 0.0
      ALFR_R = 0.0
      ALFR_W = 0.0
      ALFR_H = 0.0
      ALFRW_R = 0.0
      ALFRW_W = 0.0
      ALFRW_H = 0.0
C
      ALFI = 0.0
      ALFI_R = 0.0
      ALFI_W = 0.0
      ALFI_H = 0.0
      ALFIW_R = 0.0
      ALFIW_W = 0.0
      ALFIW_H = 0.0
C
      OK = .FALSE.
C
      IF(NOFILE) RETURN
C
      IF(LOADED) GO TO 9
C--------------------------------------------------------------------
C---- first time OSMAP is called ... load in 3-D spline data
C
      CALL GETOSFILE(OSFILE,LOSF)
      IF(LOSF.EQ.0) GO TO 800
C
      NR = 0
      NW = 0
      NH = 0
C
      LU = 31
      OPEN(UNIT=LU,FILE=OSFILE(1:LOSF),STATUS='OLD',
     &            FORM='UNFORMATTED',ERR=900)
C
      READ(LU) NR, NW, NH
C
      IF(NR.GT.NRX .OR.
     &   NW.GT.NWX .OR. 
     &   NH.GT.NHX     ) THEN
       WRITE(*,*) 'OSMAP: Array limit exceeded.'
       IF(NR.GT.NRX) WRITE(*,*) '       Increase NRX to', NR
       IF(NW.GT.NWX) WRITE(*,*) '       Increase NWX to', NW
       IF(NH.GT.NHX) WRITE(*,*) '       Increase NHX to', NH
       STOP
      ENDIF
C
      READ(LU) (RL(IR), IR=1,NR)
      READ(LU) (WL(IW), IW=1,NW)
      READ(LU) (HL(IH), IH=1,NH)
      READ(LU) (IR1(IH),IR2(IH),IW1(IH),IW2(IH), IH=1,NH)
C
      DO IC = 2, 1, -1
        DO IH=1, NH
          DO IW=IW1(IH), IW2(IH)
            READ(LU,END=5)
     &               (   A(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            READ(LU) (  AR(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            READ(LU) (  AW(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            READ(LU) (  AH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            READ(LU) ( ARW(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            READ(LU) ( ARH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            READ(LU) ( AWH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            READ(LU) (ARWH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
          ENDDO
        ENDDO
      ENDDO
C
    5 CONTINUE
      IF(IH.LT.NH) THEN
C----- only imaginary part is available
       IC1 = 2
       IC2 = 2
      ELSE
C----- both real and imaginary parts available
       IC1 = 1
       IC2 = 2
      ENDIF
      CLOSE(LU)
C
C
      RINCR = (RL(NR) - RL(1))/FLOAT(NR-1)
      WINCR = (WL(NW) - WL(1))/FLOAT(NW-1)
      LOADED = .TRUE.
C
C--------------------------------------------------------------------
    9 CONTINUE
C
C
      IF(NR.EQ.0 .OR. NW.EQ.0 .OR. NH.EQ.0) THEN
C----- map not available for some reason (OPEN or READ error on osmap.dat?)
       OK = .FALSE.
       RETURN
      ENDIF
C
C---- define specified spline coordinates
      RLSP = ALOG10(RSP)
      WLSP = ALOG10(WSP) + 0.5*RLSP
      HLSP = HSP
C
C---- assume map limits will not be exceeded
      OK = .TRUE.
C
C---- find H interval
      DO IH = 2, NH
        IF(HL(IH) .GE. HLSP) GO TO 11
      ENDDO
      IH = NH
   11 CONTINUE
C
      IF(HLSP.LT.HL(1) .OR. HLSP.GT.HL(NH)) THEN
CCC       OK = .FALSE.
CCC       WRITE(*,*) 'Over H limits.  R w H:', RSP,WSP,HSP
CCC       RETURN
       HLSP = MAX( HL(1) , MIN( HL(NH) , HLSP ) )
      ENDIF
C
C---- find R interval
      IR = INT((RLSP-RL(1))/RINCR + 2.001)
      IR1X = MAX( IR1(IH) , IR1(IH-1) )
      IR2X = MIN( IR2(IH) , IR2(IH-1) )
      IF(IR-1.LT.IR1X .OR. IR.GT.IR2X) THEN
       OK = .FALSE.
CCC       WRITE(*,*) 'Over R limits.  R w H:', RSP,WSP,HSP
CCC       RETURN
       IR = MAX( IR1X+1 , MIN( IR2X , IR ) )
       RLSP = MAX( RL(1) , MIN( RL(NR) , RLSP ) )
      ENDIF
C
C---- find W interval
      IW = INT((WLSP-WL(1))/WINCR + 2.001)
      IW1X = MAX( IW1(IH) , IW1(IH-1) )
      IW2X = MIN( IW2(IH) , IW2(IH-1) )
      IF(IW-1.LT.IW1X .OR. IW.GT.IW2X) THEN
       OK = .FALSE.
CCC       WRITE(*,*) 'Over w limits.  R w H:', RSP,WSP,HSP
CCC       RETURN
       IW = MAX( IW1X+1 , MIN( IW2X , IW ) )
       WLSP = MAX( WL(1) , MIN( WL(NW) , WLSP ) )
      ENDIF
C
      DRL = RL(IR) - RL(IR-1)
      DWL = WL(IW) - WL(IW-1)
      DHL = HL(IH) - HL(IH-1)
      TR = (RLSP - RL(IR-1)) / DRL
      TW = (WLSP - WL(IW-1)) / DWL
      TH = (HLSP - HL(IH-1)) / DHL
C
      TR = MAX( 0.0 , MIN( 1.0 , TR ) )
      TW = MAX( 0.0 , MIN( 1.0 , TW ) )
      TH = MAX( 0.0 , MIN( 1.0 , TH ) )
C
C---- compute real and imaginary parts
      DO 1000 IC = IC1, IC2
C
C---- evaluate spline in Rtheta at the corners of HL,WL cell
      DO 20 KH=1, 2
        JH = IH + KH-2
        DO 205 KW=1, 2
          JW = IW + KW-2
          A1    = A   (IR-1,JW,JH,IC)
          AR1   = AR  (IR-1,JW,JH,IC)
          AW1   = AW  (IR-1,JW,JH,IC)
          AH1   = AH  (IR-1,JW,JH,IC)
          ARW1  = ARW (IR-1,JW,JH,IC)
          ARH1  = ARH (IR-1,JW,JH,IC)
          AWH1  = AWH (IR-1,JW,JH,IC)
          ARWH1 = ARWH(IR-1,JW,JH,IC)
C
          A2    = A   (IR  ,JW,JH,IC)
          AR2   = AR  (IR  ,JW,JH,IC)
          AW2   = AW  (IR  ,JW,JH,IC)
          AH2   = AH  (IR  ,JW,JH,IC)
          ARW2  = ARW (IR  ,JW,JH,IC)
          ARH2  = ARH (IR  ,JW,JH,IC)
          AWH2  = AWH (IR  ,JW,JH,IC)
          ARWH2 = ARWH(IR  ,JW,JH,IC)
C
          DA1   = DRL*AR1   - A2   + A1
          DA2   = DRL*AR2   - A2   + A1
          DAW1  = DRL*ARW1  - AW2  + AW1
          DAW2  = DRL*ARW2  - AW2  + AW1
          DAH1  = DRL*ARH1  - AH2  + AH1
          DAH2  = DRL*ARH2  - AH2  + AH1
          DAWH1 = DRL*ARWH1 - AWH2 + AWH1
          DAWH2 = DRL*ARWH2 - AWH2 + AWH1
C
C-------- set  ALFI, dALFI/dWL, dALFI/dHL, d2ALFI/dHLdWL
          B(KW,KH)   =  (1.0-TR)* A1   + TR* A2
     &               + ((1.0-TR)*DA1   - TR*DA2  )*(TR-TR*TR)
          BW(KW,KH)  =  (1.0-TR)* AW1  + TR* AW2
     &               + ((1.0-TR)*DAW1  - TR*DAW2 )*(TR-TR*TR)
          BH(KW,KH)  =  (1.0-TR)* AH1  + TR* AH2
     &               + ((1.0-TR)*DAH1  - TR*DAH2 )*(TR-TR*TR)
          BWH(KW,KH) =  (1.0-TR)* AWH1 + TR* AWH2
     &               + ((1.0-TR)*DAWH1 - TR*DAWH2)*(TR-TR*TR)
C
C-------- also, the RL derivatives of the quantities above
          BR(KW,KH)   = (A2   - A1
     &     + (1.0-4.0*TR+3.0*TR*TR)*DA1   + (3.0*TR-2.0)*TR*DA2  )/DRL
          BRW(KW,KH)  = (AW2  - AW1
     &     + (1.0-4.0*TR+3.0*TR*TR)*DAW1  + (3.0*TR-2.0)*TR*DAW2 )/DRL
          BRH(KW,KH)  = (AH2  - AH1
     &     + (1.0-4.0*TR+3.0*TR*TR)*DAH1  + (3.0*TR-2.0)*TR*DAH2 )/DRL
          BRWH(KW,KH) = (AWH2 - AWH1
     &     + (1.0-4.0*TR+3.0*TR*TR)*DAWH1 + (3.0*TR-2.0)*TR*DAWH2)/DRL
C
  205   CONTINUE
   20 CONTINUE
C
C---- evaluate spline in  HL  at the two WL-interval endpoints
      DO 30 KW=1, 2
        B1    = B   (KW,1)
        BR1   = BR  (KW,1)
        BW1   = BW  (KW,1)
        BH1   = BH  (KW,1)
        BRW1  = BRW (KW,1)
        BRH1  = BRH (KW,1)
        BWH1  = BWH (KW,1)
        BRWH1 = BRWH(KW,1)
C
        B2    = B   (KW,2)
        BR2   = BR  (KW,2)
        BW2   = BW  (KW,2)
        BH2   = BH  (KW,2)
        BRW2  = BRW (KW,2)
        BRH2  = BRH (KW,2)
        BWH2  = BWH (KW,2)
        BRWH2 = BRWH(KW,2)
C
        DB1   = DHL*BH1   - B2   + B1
        DB2   = DHL*BH2   - B2   + B1
        DBR1  = DHL*BRH1  - BR2  + BR1
        DBR2  = DHL*BRH2  - BR2  + BR1
        DBW1  = DHL*BWH1  - BW2  + BW1
        DBW2  = DHL*BWH2  - BW2  + BW1
        DBRW1 = DHL*BRWH1 - BRW2 + BRW1
        DBRW2 = DHL*BRWH2 - BRW2 + BRW1
C
C------ set  ALFI, dALFI/dRL, dALFI/dWL
        C(KW)   =  (1.0-TH)* B1   + TH* B2
     &          + ((1.0-TH)*DB1   - TH*DB2  )*(TH-TH*TH)
        CR(KW)  =  (1.0-TH)* BR1  + TH* BR2
     &          + ((1.0-TH)*DBR1  - TH*DBR2 )*(TH-TH*TH)
        CW(KW)  =  (1.0-TH)* BW1  + TH* BW2
     &          + ((1.0-TH)*DBW1  - TH*DBW2 )*(TH-TH*TH)
        CRW(KW) =  (1.0-TH)* BRW1 + TH* BRW2
     &          + ((1.0-TH)*DBRW1 - TH*DBRW2)*(TH-TH*TH)
C
C------ also, the HL derivatives of the quantities above
        CH(KW)   = (B2   - B1
     &     + (1.0-4.0*TH+3.0*TH*TH)*DB1   + (3.0*TH-2.0)*TH*DB2  )/DHL
        CRH(KW)  = (BR2  - BR1
     &     + (1.0-4.0*TH+3.0*TH*TH)*DBR1  + (3.0*TH-2.0)*TH*DBR2 )/DHL
        CWH(KW)  = (BW2  - BW1
     &     + (1.0-4.0*TH+3.0*TH*TH)*DBW1  + (3.0*TH-2.0)*TH*DBW2 )/DHL
        CRWH(KW) = (BRW2 - BRW1
     &     + (1.0-4.0*TH+3.0*TH*TH)*DBRW1 + (3.0*TH-2.0)*TH*DBRW2)/DHL
C
   30 CONTINUE
C
C---- evaluate cubic in  WL
      C1    = C   (1)
      CR1   = CR  (1)
      CW1   = CW  (1)
      CH1   = CH  (1)
      CRW1  = CRW (1)
      CRH1  = CRH (1)
      CWH1  = CWH (1)
      CRWH1 = CRWH(1)
C
      C2    = C   (2)
      CR2   = CR  (2)
      CW2   = CW  (2)
      CH2   = CH  (2)
      CRW2  = CRW (2)
      CRH2  = CRH (2)
      CWH2  = CWH (2)
      CRWH2 = CRWH(2)
C
      DC1   = DWL*CW1   - C2   + C1
      DC2   = DWL*CW2   - C2   + C1
      DCH1  = DWL*CWH1  - CH2  + CH1
      DCH2  = DWL*CWH2  - CH2  + CH1
      DCR1  = DWL*CRW1  - CR2  + CR1
      DCR2  = DWL*CRW2  - CR2  + CR1
CC    DCRH1 = DWL*CRWH1 - CRH2 + CRH1
CC    DCRH2 = DWL*CRWH2 - CRH2 + CRH1
C
C---- set  AINT, dAINT/dRL, dAINT/dHL
      AINT(IC) =  (1.0-TW)* C1   + TW* C2
     &         + ((1.0-TW)*DC1   - TW*DC2  )*(TW-TW*TW)
      AINT_RL  =  (1.0-TW)* CR1  + TW* CR2
     &         + ((1.0-TW)*DCR1  - TW*DCR2 )*(TW-TW*TW)
      AINT_HL  =  (1.0-TW)* CH1  + TW* CH2
     &         + ((1.0-TW)*DCH1  - TW*DCH2 )*(TW-TW*TW)
C
C---- also, the WL derivatives of the quantities above
      AINT_WL  = (C2   - C1
     &     + (1.0-4.0*TW+3.0*TW*TW)*DC1   + (3.0*TW-2.0)*TW*DC2  )/DWL
      AINTW_RL = (CR2  - CR1
     &     + (1.0-4.0*TW+3.0*TW*TW)*DCR1  + (3.0*TW-2.0)*TW*DCR2 )/DWL
      AINTW_HL = (CH2  - CH1
     &     + (1.0-4.0*TW+3.0*TW*TW)*DCH1  + (3.0*TW-2.0)*TW*DCH2 )/DWL
C
      AINTW_WL = ((6.0*TW-4.0)*DC1  + (6.0*TW-2.0)*DC2 )/DWL**2
C
C
C---- convert derivatives wrt to spline coordinates (RL,WL,HL) into 
C-    derivatives wrt input variables (Rtheta,f,H)
      AINT_R(IC) = (AINT_RL + 0.5*AINT_WL) / (AL10 * RSP)
      AINT_W(IC) = (AINT_WL              ) / (AL10 * WSP)
      AINT_H(IC) =  AINT_HL
C
      AINTW_R(IC) = (AINTW_RL + 0.5*AINTW_WL) / (AL10**2 * WSP*RSP)
      AINTW_W(IC) = (AINTW_WL - AL10*AINT_WL) / (AL10**2 * WSP*WSP)
      AINTW_H(IC) =  AINTW_HL                 / (AL10    * WSP    )
C
 1000 CONTINUE
C
      ALFR = AINT(1)
      ALFR_R = AINT_R(1)
      ALFR_W = AINT_W(1)
      ALFR_H = AINT_H(1)
      ALFRW_R = AINTW_R(1)
      ALFRW_W = AINTW_W(1)
      ALFRW_H = AINTW_H(1)
C
      ALFI = AINT(2)
      ALFI_R = AINT_R(2)
      ALFI_W = AINT_W(2)
      ALFI_H = AINT_H(2)
      ALFIW_R = AINTW_R(2)
      ALFIW_W = AINTW_W(2)
      ALFIW_H = AINTW_H(2)
C
C---- if we're within the spline data space, the derivatives are valid
      IF(OK) RETURN
C
C---- if not, the  ai  value is clamped, and its derivatives are zero
      ALFR_R = 0.0
      ALFR_W = 0.0
      ALFR_H = 0.0
      ALFRW_R = 0.0
      ALFRW_W = 0.0
      ALFRW_H = 0.0
C
      ALFI_R = 0.0
      ALFI_W = 0.0
      ALFI_H = 0.0
      ALFIW_R = 0.0
      ALFIW_W = 0.0
      ALFIW_H = 0.0
C
      RETURN
C
C--------------------------------------------------------
C---- pick up here if OS file not given
 800  CONTINUE
      WRITE(*,*)'OSMAP: Environment variable OSMAP not defined'
      WRITE(*,*)'       Must be set to Orr-Sommerfeld database filename'
C---- don't try again
      NOFILE = .TRUE.
      RETURN
C
C--------------------------------------------------------
C---- pick up here for file open error
 900  CONTINUE
      WRITE(*,*)
      WRITE(*,*)'OSMAP: Orr-Sommerfeld database file not found: ', 
     &           OSFILE(1:LOSF)
      WRITE(*,*)'       Will return zero amplification rates'
C---- don't try again
      NOFILE = .TRUE.
      OK = .FALSE.
C
      RETURN
      END ! OSMAP

