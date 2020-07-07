
      SUBROUTINE NTCALC(NX,N,X,HK,TH,UE,VE, NW,W,A)
C------------------------------------------------------------------
C     Calculates range of frequencies which span the
C     critical frequency.  Also calculates the amplitude
C     distribution for each frequency.
C
C    Input:  NX     array physical dimension 
C            N      number of streamwise points i
C                    (i = N  point is assumed turbulent)
C            X (i)  streamwise coordinate array for integrating A(x)
C            HK(i)  kinematic shape parameter
C            TH(i)  momentum thickness
C            UE(i)  edge velocity
C            VE(i)  edge kinematic viscosity (in same units as UE*TH)
C            NW     number of frequencies to be set
C
C    Output: W(k)   radian frequencies in same units as UE/TH
C            A(i,k) amplitude distribution for frequency W(k)
C------------------------------------------------------------------
      REAL X(NX), HK(NX), TH(NX), UE(NX), VE(NX)
      REAL W(NW), A(NX,NW)
C
      REAL RSP,WSP,HSP,
     &       AR,
     &       AR_R, AR_W, AR_H,
     &       ARW_R,ARW_W,ARW_H ,
     &       AI,
     &       AI_R, AI_W, AI_H,
     &       AIW_R,AIW_W,AIW_H
C
      LOGICAL OK
C
C---- log(frequency) increment over range (i.e. 1.5 decades)
      DW = -1.50/FLOAT(NW-1)
C
C---- frequency and amplitude will be returned as zero if no instability
      DO 10 IW=1, NW
        W(IW) = 0.
        DO 105 I=1, N
          A(I,IW) = 0.
  105   CONTINUE
   10 CONTINUE
C
C---- search downstream for location where Rcrit is first exceeded
      DO 20 I=1, N-1
C------ local Rdelta*
        RDL = LOG10( HK(I)*TH(I)*UE(I)/VE(I) )
C
C------ approximate critical Rdelta* for local shape parameter
        HKB = 1.0 / (HK(I) - 1.0)
        RDLC = 2.23 + 1.35*HKB + 0.85*TANH(10.4*HKB - 7.07) - 0.1
C
        IF(RDL .GE. RDLC) GO TO 21
   20 CONTINUE
CCC   WRITE(*,*) 'Rcrit not exceeded'
      RETURN
C
   21 ISTART = I
C
C---- set frequency array at location where Rcrit is first exceeded
      I = ISTART
      UOT = UE(I)/TH(I)
      DO 30 IW=1, NW
        WLOG = -1.0 + DW*FLOAT(IW-1)
        W(IW) = (10.0 ** WLOG) * UOT
   30 CONTINUE
C
      DO 40 I=ISTART+1, N
        IM = I-1
C
C------ set flow variables over i-1,i interval
        UA = (UE(IM) + UE(I))*0.5
        VA = (VE(IM) + VE(I))*0.5
        TA = (TH(IM) + TH(I))*0.5
        HA = (HK(IM) + HK(I))*0.5
C
        IF(I.EQ.N) THEN
C------- last point is turbulent, so extrapolate from laminar region
C-       (turbulent H is likely to be inappropriate)
         UA = 1.5*UE(IM) - 0.5*UE(IM-1)
         VA = 1.5*VE(IM) - 0.5*VE(IM-1)
         TA = 1.5*TH(IM) - 0.5*TH(IM-1)
         HA = 1.5*HK(IM) - 0.5*HK(IM-1)
        ENDIF
C
C------ set local Rtheta, Hk
        RSP = UA*TA/VA
        HSP = HA
C
C------ limit Hk, (OSMAP routine would clip anyway)
        HSP = MIN( HSP , 19.999 )
C
C------ go over frequencies
        DO 405 IW=1, NW
C-------- set Ue/Theta-normalized frequency  WSP = w Theta/Ue
          WSP = W(IW)*TA/UA
C
C-------- calculate Theta-normalized spatial growth rate  AI = ai * Theta
          CALL OSMAP(RSP,WSP,HSP,
     &               AR,
     &               AR_R, AR_W, AR_H,
     &               ARW_R,ARW_W,ARW_H ,
     &               AI,
     &               AI_R, AI_W, AI_H,
     &               AIW_R,AIW_W,AIW_H , OK)
C
C-------- integrate growth rate to get amplitude
          DX = X(I) - X(IM)
          A(I,IW) = A(IM,IW) - AI * DX/TA
          A(I,IW) = MAX( A(I,IW) , 0.0 )
  405   CONTINUE
   40 CONTINUE
C
      RETURN
      END

