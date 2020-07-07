
      PARAMETER (NHX=8, NRX=40)
      DIMENSION H(NHX), R(0:NRX)
      CHARACTER*5 HNAME(NHX)
      LOGICAL OK
C
      DATA H / 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 12.0, 20.0 /
      DATA HNAME / 'ai.03' ,
     &             'ai.35' ,
     &             'ai.04' ,
     &             'ai.05' ,
     &             'ai.06' ,
     &             'ai.08' ,
     &             'ai.12' ,
     &             'ai.20'   /
C
      FSP = 0.08
C
      DO IR=0, NRX
        R(IR) = 10.0 ** (FLOAT(IR)/10.0)
      ENDDO
C
      DO IH = 1, NHX
C
        OPEN(1,FILE=HNAME(IH),STATUS='unknown')
        WRITE(1,1000) '#', H(IH)
 1000   FORMAT(A, F10.5)
C
        DO IR=1, NRX
          CALL OSHAI(R(IR),FSP,H(IH), AI,
     &               AI_R, AI_F, AI_H,
     &               AIF_R,AIF_F,AIF_H , OK)
C
          WRITE(1,1200) LOG10(R(IR)), -AI
 1200     FORMAT(1X,F10.5, F12.6)
C
        ENDDO
        CLOSE(1)
      ENDDO
C
      STOP
      END

