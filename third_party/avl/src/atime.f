C***********************************************************************
C    Module:  atime.f
C 
C    Copyright (C) 2002 Mark Drela, Harold Youngren
C 
C    This program is free software; you can redistribute it and/or modify
C    it under the terms of the GNU General Public License as published by
C    the Free Software Foundation; either version 2 of the License, or
C    (at your option) any later version.
C
C    This program is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
C    GNU General Public License for more details.
C
C    You should have received a copy of the GNU General Public License
C    along with this program; if not, write to the Free Software
C    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C***********************************************************************

      SUBROUTINE TIME
C---------------------------------------------
C     Time-domain integration driver
C---------------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      LOGICAL ERROR, LOK, LWRIT, SAVED, OVERLAY
C
      CHARACTER*1 ITEM, ANS, CHKEY
      CHARACTER*2 OPT, CHPLOT
      CHARACTER*4 COMAND, ITEMC
      CHARACTER*80 FNOUT, FNNEW, FNSYS
      CHARACTER*80 LINE, FNVB, COMARG, PROMPT, RTNEW
      CHARACTER SATYPE*50, ROTTYPE*50
C
      REAL*8 ASYS(JETOT,JETOT),BSYS(JETOT,NDMAX),RSYS(JETOT)
C
      PARAMETER (JTMAX=JETOT+NDMAX)
      REAL AMAT(JTMAX,JTMAX),RMAT(JTMAX),WORK(JTMAX)
      REAL DTPAR(JEMAX), DTCON(NDMAX)
      INTEGER IMAT(JTMAX)
C
      REAL RINPUT(20), RINP(20)
      INTEGER IINPUT(20), IINP(20)
C
      IRUN0 = IRUN
C
      FNVB = ' '
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
      IF(LSA_RATES) THEN
        ROTTYPE = 'Rates,moments about Stability axes'
       ELSE
        ROTTYPE = 'Rates,moments about Body axes'
      ENDIF
C
 1000 FORMAT (A)
C
C---- force computation of plot limits
      TMIN = 0.
      TMAX = 0.
      FMIN = 0.
      FMAX = 0.
C
C
      LPLOT = .FALSE.
      LWRIT = .FALSE.
C
      FNOUT = ' '
C
      CHPLOT = '  '
C
c      EYEPTZ = 180.
c      EYEPTY = 90.
c      EYEPTX = 0.
c      ROBINV = 0.
C
      OVERLAY = .FALSE.
C
C=================================================================
C---- start of user interaction loop
 800  CONTINUE
C
      WRITE(*,1050) 
 1050 FORMAT(
     &  /'  Run-case parameters for candidate initial states... ')
C
      LU = 6
      CALL RUNLST(LU,IRUN)
C
C
C
      WRITE(*,1052)
 1052 FORMAT(
     &   ' =========================================================='
     & //' "#" select run case for initial state'
     &  /'  M odify parameters'
     & //' eX ecute time march calculation'
     & //'  V iew time march calculation'
     &  /'  P lot time traces'
     &  /'  W rite time traces to screen or file'
     &  /'  D ata file overlay toggle'
     & //'  Z oom'
     &  /'  U nzoom')
C
C   A B C D E F G H I J K L M N O P Q R S T U V W X Y Z
C         x                 x     x         x x x x   x

 810  CONTINUE
      CALL ASKC(' .TIME^',COMAND,COMARG)
C
 815  CONTINUE
C------------------------------------------------------
      IF    (COMAND.EQ.'    ') THEN
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL CLRZOOM
       IRUN = IRUN0
       RETURN
C
      ELSEIF(COMAND.EQ.'?   ') THEN
       GO TO 800
C
      ENDIF
C
C-------------------------------------------------------------------
C---- see if command is an integer
      NINPUT = 1
      CALL GETINT(COMAND,IINPUT,NINPUT,ERROR)
      IF(.NOT.ERROR .AND. NINPUT.GE.1 
     &  .AND. COMAND(1:1).NE.'T'
     &  .AND. COMAND(1:1).NE.'F' ) THEN
C----- command is an integer... new case index?
       IF(IINPUT(1).LT.0 .OR. IINPUT(1).GT.NRUN) THEN
        WRITE(*,*)
        WRITE(*,*) '* Selected new run case is not defined'
        GO TO 800
       ELSE
C------ valid new run case selected... go back to menu
        IRUN = IINPUT(1)
        GO TO 800
       ENDIF
      ENDIF
C
C-------------------------------------------------------------------
C---- extract command line numeric arguments
      DO I=1, 20
        IINPUT(I) = 0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 20
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 20
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
C-----------------------------------------------------------------------
      IF    (COMAND.EQ.'M   ') THEN
       CALL PARMOD(IRUN)
C
C-------------------------------------------------------------------
      ELSEIF(COMAND .EQ. 'X   ') THEN
C------ execute time march calculation
C
 22     RINPUT(1) = DELTAT
        RINPUT(2) = FLOAT(NTSTEPS)
        WRITE(*,2022) DELTAT, NTSTEPS
 2022   FORMAT(
     &   /' Enter delta(t), Nstep  (-Nstep to append steps)', F9.5, I8)
        CALL READR(2,RINPUT,ERROR)
        IF(ERROR) GO TO 22
C
C------ abort?
        IF(RINPUT(1).LE.0.0 .OR. RINPUT(2).EQ.0.0) GO TO 510
C
        DELTAT = RINPUT(1)
        NTSTEPS = INT( RINPUT(2) + SIGN(0.01,RINPUT(2)) )
        NTSTEP  = IABS(NTSTEPS)
C
C------ append to current time trace?
        LTAPP = NTSTEPS .LT. 0
C
C------ set up marching indices and initial state
        IF(LTAPP) THEN
C------- make sure we have a valid time trace up to now
         IF(NTLEV.LT.2) THEN
C--------- bug out
           WRITE(*,*) 'No time march available to append to.'
           GO TO 510
         ENDIF
C
C------- will just add NTSTEP points starting at current time level ITLEV
C-       (any levels after ITLEV will be overwritten)
         ITLEV1 = ITLEV
         ITLEV2 = ITLEV + NTSTEP
C
        ELSE
C------- will start new sequence from current run case IRUN
         ITLEV1 = 1
         ITLEV2 = 1 + NTSTEP
C
         IR = IRUN
C
         CALL RUNCHK(IR,LOK)
         IF(.NOT.LOK) THEN
          WRITE(*,*) '** No initial state.  Ill-posed run case', IR
          GO TO 100
         ENDIF
C
C------- converge initial state
         NITER = 10
         INFO = 0
         CALL EXEC(NITER,INFO,IR)
C
         VEE = PARVAL(IPVEE,IR)
         ROT = VEE/UNITL
C
         RHO = PARVAL(IPRHO,IR)
         QS = 0.5*RHO*VEE**2 * SREF*UNITL**2
C
         WEIGHT = PARVAL(IPMASS,IR)*PARVAL(IPGEE,IR)
C
C------- set initial state
         IT = ITLEV1
         TLEV(IT) = 0.
         TPARS(KPBANK,IT) = PARVAL(IPPHI,IR)*DTR
         TPARS(KPELEV,IT) = PARVAL(IPTHE,IR)*DTR
         TPARS(KPHEAD,IT) = PARVAL(IPPSI,IR)*DTR
         TPARS(KPVINF,IT) = PARVAL(IPVEE ,IR)
         TPARS(KPALFA,IT) = PARVAL(IPALFA,IR)*DTR
         TPARS(KPBETA,IT) = PARVAL(IPBETA,IR)*DTR
         TPARS(KPCLIF,IT) = CLTOT
         TPARS(KPCDRG,IT) = CDTOT
         TPARS(KPCDIN,IT) = CDITOT
         TPARS(KPCMOM,IT) = CMTOT
         TPARS(KPLIFT,IT) = CLTOT*QS
         TPARS(KPDRAG,IT) = CDTOT*QS
         TPARS(KPMACH,IT) = 0.0
         TPARS(KPALTK,IT) = 0.0
         TPARS(KPDENS,IT) = RHO
         TPARS(KPVSOU,IT) = 1.0
         TPARS(KPLOAD,IT) = CLTOT*QS / WEIGHT
C
         TPARV(1,KPVEL,IT) =  VINF(1)*VEE
         TPARV(2,KPVEL,IT) = -VINF(2)*VEE
         TPARV(3,KPVEL,IT) =  VINF(3)*VEE
         TPARV(1,KPROT,IT) = -WROT(1)*ROT
         TPARV(2,KPROT,IT) =  WROT(2)*ROT
         TPARV(3,KPROT,IT) = -WROT(3)*ROT
         TPARV(1,KPPOS,IT) = 0.0
         TPARV(2,KPPOS,IT) = 0.0
         TPARV(3,KPPOS,IT) = 0.0
         TPARV(1,KPFOR,IT) = CXTOT
         TPARV(2,KPFOR,IT) = CYTOT
         TPARV(3,KPFOR,IT) = CZTOT
         TPARV(1,KPMOM,IT) = CRTOT
         TPARV(2,KPMOM,IT) = CMTOT
         TPARV(3,KPMOM,IT) = CNTOT
C 
         DO N = 1, NCONTROL
           TPARD(N,IT) = DELCON(N)
         ENDDO
        ENDIF
C
C
        DO 50 ITLEV = ITLEV1+1, ITLEV2
          IT  = ITLEV
          ITM = ITLEV - 1
          ITL = MAX( 1 , ITLEV - 2 )
C
C-------- initial guess for current time from previous time
          CALL TCOPY(ITLEV-1,ITLEV)
C
          TLEV(ITLEV) = TLEV(ITLEV-1) + DELTAT
C
cC-------- better initial guess by extrapolating in time
c          IF(IPTIME.GT.2) CALL QPRED(IPTIME)
c          PSTEADY(IPTIME) = .FALSE.
C
C-------- set time-differencing weights
          CALL TDSET(ITLEV,TLEV,TDER)
C
          WRITE(*,*)
          WRITE(*,*)
     &       'Converging time level',ITLEV,' ... t = ',TLEV(ITLEV)
C


          INFO = 1
          ETOL = 1.0E-5
          IF(COMAND .EQ. 'X   ') THEN
           CALL SYSMAT(IR,ASYS,BSYS,RSYS,NSYS)
          ELSE
           CALL APPMAT(IR,ASYS,BSYS,RSYS,NSYS)
          ENDIF
C
cc          LU = 6
cc          CALL SYSSHO(LU,ASYS,BSYS,RSYS,NSYS)
C
          NMAT = JETOT + NCONTROL
          DO K = 1, NMAT
            RMAT(K) = 0.
            DO J = 1, NMAT
              AMAT(K,J) = 0.
            ENDDO
          ENDDO
C
C-------- set total residuals and Jacobian
          DO K = 1, JETOT
            RMAT(K) = RSYS(K)
     &              - TDER(1)*TPAR(K,IT )
     &              - TDER(2)*TPAR(K,ITM)
     &              - TDER(3)*TPAR(K,ITL)
            DO J = 1, JETOT
              AMAT(K,J) = ASYS(K,J) - TDER(1)
            ENDDO
            DO L = 1, NCONTROL
              J = JETOT + L
              AMAT(K,J) = BSYS(K,L)
            ENDDO
          ENDDO
C
C-------- set control-variable residuals and Jacobian
          DO N = 1, NCONTROL
            K = JETOT + N
            RMAT(K) = TCON(N,IT) - TCONSP(N,IT)
            AMAT(K,K) = 1.0
          ENDDO
C
          CALL LUDCMP(JTMAX,NMAT,AMAT,IMAT,WORK)
          CALL BAKSUB(JTMAX,NMAT,AMAT,IMAT,RMAT)
C
          DRMS = 0.
          DMAX = 0.
          JMAX = 0
          DO J = 1, JETOT
            DTPAR(J) = -RMAT(J)
C
            IF(ABS(DTPAR(J))/TREF(J) .GT. ABS(DMAX)) THEN
             DMAX = DTPAR(J)
             JMAX = J
            ENDIF
            DRMS = DRMS + (DTPAR(J)/TREF(J))**2
          ENDDO
          DO N = 1, NCONTROL
            J = JETOT + N
            DTCON(N) = -RMAT(J)
C
            IF(ABS(DTCON(N))/TREF(J) .GT. ABS(DMAX)) THEN
             DMAX = DTCON(N)
             JMAX = J
            ENDIF
            DRMS = DRMS + (DTCON(N)/TREF(J))**2
          ENDDO
C
          DRMS = SQRT( DRMS/FLOAT(JETOT+NCONTROL) )
C
C
          RLX = 1.0
          DO J = 1, JETOT
            TPAR(J,IT) = TPAR(J,IT) + RLX*DTPAR(J)
          ENDDO
          DO N = 1, NCONTROL
            TCON(N,IT) = TCON(N,IT) + RLX*DTCON(N)
          ENDDO
C
          VSQ = TPAR(JUE,IT)**2 + TPAR(JVE,IT)**2 + TPAR(JWE,IT)**2
          VEE = SQRT(VSQ)
          ROT = VEE/UNITL
C
          IF(VEE .EQ. 0.0) THEN
           VINV = 1.0
           RINV = 1.0
          ELSE
           VINV = 1.0/VEE
           RINV = 1.0/ROT
          ENDIF
C
          VINF(1) =  TPAR(JEU ,IT)*VINV
          VINF(2) = -TPAR(JEV ,IT)*VINV
          VINF(3) =  TPAR(JEW ,IT)*VINV
          WROT(1) = -TPAR(JEP ,IT)*RINV
          WROT(2) =  TPAR(JEQ ,IT)*RINV
          WROT(3) = -TPAR(JER ,IT)*RINV
C
          V13 = SQRT( VINF(1)**2 + VINF(3)**2 )
C
          ALFA = ATAN2(  VINF(3) , VINF(1) )
          BETA = ATAN2( -VINF(2) , V13     )
C
          PARVAL(IPALFA,IR) = ALFA/DTR
          PARVAL(IPBETA,IR) = BETA/DTR
          PARVAL(IPROTX,IR) = WROT(1)*0.5*BREF
          PARVAL(IPROTY,IR) = WROT(2)*0.5*CREF
          PARVAL(IPROTZ,IR) = WROT(3)*0.5*BREF
          PARVAL(IPCL  ,IR) = CLTOT
C
          PARVAL(IPALFA,IR) = ALFA
          PARVAL(IPBETA,IR) = BETA
          PARVAL(IPROTX,IR) = WROT(1)
          PARVAL(IPROTY,IR) = WROT(2)
          PARVAL(IPROTZ,IR) = WROT(3)
          PARVAL(IPPHI,IR) = TPAR(JEPH,IT)
          PARVAL(IPTHE,IR) = TPAR(JETH,IT)
          PARVAL(IPPSI,IR) = TPAR(JEPS,IT)
C
C-------- sum AIC matrices to get GAM,SRC,DBL
          CALL GAMSUM
C
C-------- sum AIC matrices to get WC,WV
          CALL VELSUM
C
C-------- compute forces
          CALL AERO
C
C
          IF(DRMS .LT. EPS .AND. ABS(DMAX) .LT. 10.0*EPS) THEN
           GO TO 100
          ENDIF


C
 100    CONTINUE
C
        CHPLOT = 'P '
C
C-------------------------------------------------------------------
C---- Blowup window
      ELSEIF(COMAND.EQ.'B   ') THEN
       IF(INDEX('P',CHPLOT(1:1)) .EQ. 0) THEN
        WRITE(*,*) 'No plot to blow up'
        GO TO 810
       ENDIF
C
       DCROSS = 2.0
       WRITE(*,*) 'Mark off corners of blowup area'
       CALL GETCURSORXY(XC1,YC1,CHKEY)
       CALL PLSYMB(XC1,YC1,DCROSS,3,0.0,0)
       CALL PLFLUSH
       CALL GETCURSORXY(XC2,YC2,CHKEY)
       CALL PLSYMB(XC2,YC2,DCROSS,3,0.0,0)
       CALL PLFLUSH
C
       XE1 = TMIN+(XC1-XORG0)/TFAC
       YE1 = FMIN+(YC1-YORG0)/FFAC
       XE2 = TMIN+(XC2-XORG0)/TFAC
       YE2 = FMIN+(YC2-YORG0)/FFAC
C
       TMIN = MIN( XE1 , XE2 )
       TMAX = MAX( XE1 , XE2 )
       FMIN = MIN( YE1 , YE2 )
       FMAX = MAX( YE1 , YE2 )
C
       CALL AXISADJ(TMIN,TMAX, TTOT, TDEL, NANN)
       CALL AXISADJ(FMIN,FMAX, FTOT, FDEL, NANN)
C
       COMAND = 'P   '
       GO TO 815
C
C-------------------------------------------------------------------
C---- Normal size
      ELSEIF(COMAND.EQ.'R   ') THEN
       TMIN = 0.
       TMAX = 0.
       FMIN = 0.
       FMAX = 0.
C
       COMAND = 'P   '
       GO TO 815
C
C-------------------------------------------------------------------
C---- Annotate
      ELSEIF(COMAND.EQ.'A   ') THEN
       IF(LPLOT) THEN
        CALL ANNOT(CH)
       ELSE
        WRITE(*,*) 'No active plot'
       ENDIF
C
C-------------------------------------------------------------------
C---- Hardcopy
      ELSEIF(COMAND.EQ.'H   ') THEN
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL REPLOT(IDEVH)
C
C-------------------------------------------------------------------
C---- write eigenvalues
      ELSEIF(COMAND.EQ.'W   ') THEN
       IF(FEVDEF(1:1).EQ.' ') THEN
C------ set default filename
        KDOT = INDEX(FILDEF,'.')
        IF(KDOT.EQ.0) THEN
         CALL SLEN(FILDEF,NFIL)
         FEVDEF = FILDEF(1:NFIL) // '.eig'
        ELSE
         FEVDEF = FILDEF(1:KDOT) // 'eig'
        ENDIF
       ENDIF
C
       CALL SLEN(FEVDEF,NFE)
       NFE = MAX( NFE , 1 )
       WRITE(*,2040) FEVDEF(1:NFE)
 2040  FORMAT(' Enter eigenvalue save filename: ', A)
       READ (*,1000) FNNEW
       IF(FNNEW.NE.' ') FEVDEF = FNNEW
       CALL EIGOUT(FEVDEF, IRUN1,IRUN2, SAVED)
C
C-------------------------------------------------------------------
C---- toggle overlay flag
      ELSEIF(COMAND.EQ.'D   ') THEN
       OVERLAY = .NOT.OVERLAY
C
       IF(OVERLAY) THEN
        IF(FEVDEF(1:1).EQ.' ') THEN
C------- set default filename
         KDOT = INDEX(FILDEF,'.')
         IF(KDOT.EQ.0) THEN
          CALL SLEN(FILDEF,NFIL)
          FEVDEF = FILDEF(1:NFIL) // '.eig'
         ELSE
          FEVDEF = FILDEF(1:KDOT) // 'eig'
         ENDIF
        ENDIF
C
        CALL SLEN(FEVDEF,NFE)
        NFE = MAX( NFE , 1 )
        WRITE(*,2050) FEVDEF(1:NFE)
 2050   FORMAT(' Enter eigenvalue data filename: ', A)
        READ (*,1000) FNNEW
        IF(FNNEW.NE.' ') FEVDEF = FNNEW
C
C------ read the data file
        CALL EIGINP(FEVDEF, ERROR)
        IF(ERROR) THEN
         OVERLAY = .FALSE.
         GO TO 800
        ENDIF
C
        OVERLAY = .TRUE.
        WRITE(*,*) 'Overlay plotting enabled'
C
        IF(INDEX('P',CHPLOT(1:1)).NE.0) THEN
         COMAND = CHPLOT
         GO TO 815
        ENDIF
C
       ELSE
        WRITE(*,*) 'Overlay plotting disabled'
C
       ENDIF
C
C-------------------------------------------------------------------
C---- Zoom in on plot
      ELSEIF(COMAND.EQ.'Z   ') THEN
       IF(INDEX('P',CHPLOT(1:1)) .EQ. 0) THEN
        WRITE(*,*) 'No plot to zoom'
        GO TO 810
       ENDIF
C
       CALL USETZOOM(.TRUE.,.TRUE.)
       CALL REPLOT(IDEV)
C
C-------------------------------------------------------------------
C---- Reset zoom on plot
      ELSEIF(COMAND.EQ.'U   ') THEN
       CALL CLRZOOM
       CALL REPLOT(IDEV)
C
C-------------------------------------------------------------------
      ELSE
        WRITE(*,*)
        WRITE(*,*) '* Option not recognized'
C
      ENDIF
      GO TO 800
C
      END ! TIME



      SUBROUTINE TCOPY(IT1,IT2)
      INCLUDE 'AVL.INC'
C
      TLEV(IT2) = TLEV(IT1)
      DO J = 1, JETOT
        TPAR(J,IT2) = TPAR(J,IT1)
      ENDDO
      DO N = 1, NCONTROL
        TCON(N,IT2) = TCON(N,IT1)
      ENDDO
C
      RETURN
      END ! TCOPY



      SUBROUTINE TDSET(ITIME,TIME,QTWT)
      REAL TIME(*), QTWT(3)
C
      REAL QTWT1(3), QTWT2(3)
C
C---- set differencing blending fraction
C
C-      F = 0   1st-order backward difference
C-      F = 1   2nd-order backward difference
C 
      DATA F / 1.0 /
ccc      DATA F / 0.0 /
C
      IF    (ITIME.LT.2) THEN
        QTWT1(1) = 0.
        QTWT1(2) = 0.
        QTWT1(3) = 0.
        QTWT2(1) = 0.
        QTWT2(2) = 0.
        QTWT2(3) = 0.
C
      ELSEIF(ITIME.EQ.2) THEN
        T0 = TIME(ITIME  )
        T1 = TIME(ITIME-1)
C
        QTWT1(1) =  1.0/(T0-T1)
        QTWT1(2) = -1.0/(T0-T1)
        QTWT1(3) = 0.
C
        QTWT2(1) =  1.0/(T0-T1)
        QTWT2(2) = -1.0/(T0-T1)
        QTWT2(3) = 0.
C
      ELSE
        T0 = TIME(ITIME  )
        T1 = TIME(ITIME-1)
        T2 = TIME(ITIME-2)
C
        QTWT1(1) =  1.0/(T0-T1)
        QTWT1(2) = -1.0/(T0-T1)
        QTWT1(3) = 0.
C
        QTWT2(1) =  1.0/(T0-T1) + 1.0/(T0-T2)
        QTWT2(2) = -1.0/(T0-T1) - 1.0/(T0-T2)
     &             -    (T0-T1)/((T0-T2)*(T1-T2))
        QTWT2(3) =      (T0-T1)/((T0-T2)*(T1-T2))
C
      ENDIF
C
C---- blend 1st and 2nd order differences
      QTWT(1) = (1.0-F)*QTWT1(1) + F*QTWT2(1)
      QTWT(2) = (1.0-F)*QTWT1(2) + F*QTWT2(2)
      QTWT(3) = (1.0-F)*QTWT1(3) + F*QTWT2(3)
C
      RETURN
      END ! TDSET

