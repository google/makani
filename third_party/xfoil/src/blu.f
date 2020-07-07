
      PROGRAM BLU
      REAL MACH, MSQ
      LOGICAL TURB
C
      PARAMETER (KPRX=129)
      REAL XX(KPRX), YY(KPRX), FFS(KPRX), SFS(KPRX)
C
      CHARACTER*80 ARGP1,ARGP2,ARGP3,ARGP4,ARGP5,ARGP6,ARGP7

      CALL GETARG(1,ARGP1)
      CALL GETARG(2,ARGP2)
      CALL GETARG(3,ARGP3)
      CALL GETARG(4,ARGP4)
      CALL GETARG(5,ARGP5)
      CALL GETARG(6,ARGP6)

      IF(ARGP1 .EQ. ' ') THEN
       WRITE(*,*) 'Usage:'
       WRITE(*,*) '% blu  Hk [ Rtheta theta Ue Me n ]'
       STOP
      ELSE
       READ(ARGP1,*) HK
      ENDIF

      IF(ARGP2 .EQ. ' ') THEN
       RET = 0.
      ELSE
       READ(ARGP2,*) RET
      ENDIF

      IF(ARGP3 .EQ. ' ') THEN
       TH = 1.0
      ELSE
       READ(ARGP3,*) TH
      ENDIF

      IF(ARGP4 .EQ. ' ') THEN
       UE = 1.0
      ELSE
       READ(ARGP4,*) UE
      ENDIF

      IF(ARGP5 .EQ. ' ') THEN
       MSQ = 0.
      ELSE
       READ(ARGP5,*) MACH
       MSQ = MACH**2
      ENDIF

      IF(ARGP6 .EQ. ' ') THEN
       NN = KPRX
      ELSE
       READ(ARGP6,*) NN
       NN = MIN( NN , KPRX )
      ENDIF

      UO = 1.0
      DK = HK*TH
      CT = 0.
C
      IF(RET .GT. 0.0) THEN
C------ set Spalding + power-law turbulent profile
        CALL PRWALL(DK,TH,UO,RET,MSQ,CT, BB,
     &        DE, DE_DS, DE_TH, DE_UO, DE_RT, DE_MS,
     &        US, US_DS, US_TH, US_UO, US_RT, US_MS,
     &        HS, HS_DS, HS_TH, HS_UO, HS_RT, HS_MS,
     &        CF, CF_DS, CF_TH, CF_UO, CF_RT, CF_MS,
     &        CD, CD_DS, CD_TH, CD_UO, CD_RT, CD_MS,
     &            CD_CT  )
        CALL UWALL(TH,UO,DE,US,RET,CF,BB,   YY,XX,NN)

        DO K=1, NN
          XX(K) = XX(K)*UE
        ENDDO
C
      ELSE
C------ set Falkner-Skan profile
        INORM = 3
        ISPEC = 2
        HSPEC = HK
        ETAE = 1.5*(3.15 + 1.72/(HK-1.0) + HK) + 2.0 
        GEO = 1.0
        CALL FS(INORM,ISPEC,BU,HSPEC,NN,ETAE,GEO,YY,FFS,XX,SFS,DEFS)
        DE = ETAE*TH
        DO K=1, NN
          XX(K) = XX(K)*UE
          YY(K) = YY(K)*TH
        ENDDO
C
      ENDIF

      DO K = 1, NN
        WRITE(*,'(1X,2G13.5)') YY(K), XX(K)
      ENDDO

      STOP
      END



      SUBROUTINE CFT( HK, RT, MSQ, CF, CF_HK, CF_RT, CF_MSQ )
      IMPLICIT REAL (A-H,M,O-Z)
      INCLUDE 'BLPAR.INC'
C
      DATA GAM /1.4/
C
C---- Turbulent skin friction function  ( Cf )    (Coles)
      GM1 = GAM - 1.0
      FC = SQRT(1.0 + 0.5*GM1*MSQ)
      GRT = LOG(RT/FC)
      GRT = MAX(GRT,3.0)
C
      GEX = -1.74 - 0.31*HK
C
      ARG = -1.33*HK
      ARG = MAX(-20.0, ARG )
C
      THK = TANH(4.0 - HK/0.875)
C
      CFO =  CFFAC * 0.3*EXP(ARG) * (GRT/2.3026)**GEX
      CF     = ( CFO  +  1.1E-4*(THK-1.0) ) / FC
      CF_HK  = (-1.33*CFO - 0.31*LOG(GRT/2.3026)*CFO
     &         - 1.1E-4*(1.0-THK**2) / 0.875    ) / FC
      CF_RT  = GEX*CFO/(FC*GRT) / RT
      CF_MSQ = GEX*CFO/(FC*GRT) * (-0.25*GM1/FC**2) - 0.25*GM1*CF/FC**2
C
      RETURN
      END ! CFT

