C***********************************************************************
C    Module:  amode.f
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

      SUBROUTINE MODE
C---------------------------------------------
C     Flight dynamics analysis driver
C---------------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      LOGICAL ERROR, LOK, LWRIT, SAVED, OVERLAY, LOWRIT
C
      CHARACTER*1 ITEM, ANS, CHKEY
      CHARACTER*2 OPT, CHPLOT
      CHARACTER*4 COMAND, ITEMC
      CHARACTER*80 FNOUT, FNNEW, FNSYS
      CHARACTER*80 LINE, FNVB, COMARG, PROMPT, RTNEW
      CHARACTER SATYPE*50, ROTTYPE*50
C
      LOGICAL LPROOT(JEMAX,NRMAX),
     &        LPRNUM(JEMAX,NRMAX)
      SAVE LPROOT,LPRNUM
C
      REAL*8 ASYS(JEMAX,JEMAX),BSYS(JEMAX,NDMAX),RSYS(JEMAX)
      COMPLEX ZCROOT
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
C---- no viewed eigenmode yet
      KEVIEW = 0
      IRVIEW = 0
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


      write(*,*) 3, parnam(ipixx), '  ', parunch(ipixx)

C
C=================================================================
C---- start of user interaction loop
 800  CONTINUE
C
      WRITE(*,1050) 
 1050 FORMAT(
     &  /'  Run-case parameters for eigenmode analyses ... ')
C
      LU = 6
      CALL RUNLST(LU,IRUNE)
C
C
C
      WRITE(*,1052) LSVMOV
 1052 FORMAT(
     &   ' =========================================================='
     & //' "#" select run case for eigenmode analysis (0 = all)'
     & //'  M odify parameters'
     & //'  N ew eigenmode calculation'
     & //'  P lot root locus'
     &  /'  B lowup window'
     &  /'  R eset to normal size'
     &  /' eX amine selected eigenmode'
     & //'  A nnotate current plot'
     &  /'  H ardcopy current plot'
     &  /'  T ime-integration parameters'
     &  /'  G enerate hardcopy movie toggle', L3
     & //'  S ystem matrix output'
     &  /'  W rite eigenvalues to file'
     &  /'  D ata file overlay toggle'
     & //'  Z oom'
     &  /'  U nzoom')
C
C   A B C D E F G H I J K L M N O P Q R S T U V W X Y Z
C   x x x x     x x         x x   x   x x x x   x x   x

 810  CONTINUE
      CALL ASKC(' .MODE^',COMAND,COMARG)
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
        IRUNE = IINPUT(1)
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
C---- set up for multiple or single run cases
      IF(IRUNE.EQ.0) THEN
       IRUN1 = 1
       IRUN2 = NRUN
      ELSE
       IRUN1 = IRUNE
       IRUN2 = IRUNE
      ENDIF
C
C-----------------------------------------------------------------------
c      IF    (COMAND.EQ.'R   ') THEN
cC----- set moment reference point to CG location
c       DXCREF = (XYZMASS(1)/UNITL - XYZREF(1)) / CREF
c       WRITE(*,*)
c       WRITE(*,1210) '  Previous XYZref  = ', (XYZREF(K), K=1,3),'Lunit'
c       XYZREF(1) = XYZMASS(1)/UNITL
c       XYZREF(2) = XYZMASS(2)/UNITL
c       XYZREF(3) = XYZMASS(3)/UNITL
c       WRITE(*,1210) '  New (CG) XYZref  = ', (XYZREF(K), K=1,3),'Lunit'
cC
c       WRITE(*,*)
c       WRITE(*,1210) '  delta(Xref)/Cref = ', DXCREF
c 1210  FORMAT(1X, A, 3G10.4,3X,A)
cC
C-----------------------------------------------------------------------
      IF    (COMAND.EQ.'M   ') THEN
       CALL PARMOD(IRUNE)
C
C-------------------------------------------------------------------
      ELSEIF(COMAND .EQ. 'N   ' .OR.
     &       COMAND .EQ. 'C   '      ) THEN
C------ execute eigenmode calculation
        DO 100 IR = IRUN1, IRUN2
          CALL RUNCHK(IR,LOK)
          IF(.NOT.LOK) THEN
           WRITE(*,*) '** Skipping ill-posed run case', IR
           GO TO 100
          ENDIF
C
          NITER = 10
          INFO = 1
          CALL EXEC(NITER,INFO,IR)
C
          IF(COMAND .EQ. 'N   ') THEN
           CALL SYSMAT(IR,ASYS,BSYS,RSYS,NSYS)
          ELSE
           CALL APPMAT(IR,ASYS,BSYS,RSYS,NSYS)
          ENDIF
C
cc          LU = 6
cc          CALL SYSSHO(LU,ASYS,BSYS,RSYS,NSYS)
C
          INFO = 1
          ETOL = 1.0E-5
          CALL EIGSOL(INFO,IR,ETOL,ASYS,NSYS)
C
C
          DO KEIG = 1, NEIGEN(IR)
            LPROOT(KEIG,IR) = .TRUE.
            LPRNUM(KEIG,IR) = IRUN1 .NE. IRUN2
          ENDDO
C
          XORG0 = 0.0
          YORG0 = 0.0
          TMIN = 0.
          TMAX = 0.
          FMIN = 0.
          FMAX = 0.
          CALL PLEMAP(XORG0,YORG0, 
     &                IRUN1,IR,
     &                KEVIEW,IRVIEW,
     &                LPROOT,LPRNUM,OVERLAY,
     &                TMIN,TMAX,TDEL,TFAC,
     &                FMIN,FMAX,FDEL,FFAC )
C
          WRITE(*,*)
          CALL EIGLST(6,IR)
C
 100    CONTINUE
C
        CHPLOT = 'P '
C
C-------------------------------------------------------------------
      ELSEIF(COMAND .EQ. 'P   ') THEN
        DO IR = IRUN1, IRUN2
          IF(NEIGEN(IR) .GT. 0) GO TO 51
        ENDDO
        WRITE(*,*) 'First compute eigenmodes with N command'
        GO TO 810
C
 51     XORG0 = 0.0
        YORG0 = 0.0
        CALL PLEMAP(XORG0,YORG0, 
     &              IRUN1,IRUN2,
     &              KEVIEW,IRVIEW,
     &              LPROOT,LPRNUM,OVERLAY,
     &              TMIN,TMAX,TDEL,TFAC,
     &              FMIN,FMAX,FDEL,FFAC )
        CHPLOT = COMAND(1:2)
C
C-------------------------------------------------------------------
      ELSEIF(COMAND .EQ. 'X   ') THEN
       DO IR = IRUN1, IRUN2
         IF(NEIGEN(IR) .GT. 0) GO TO 61
       ENDDO
       WRITE(*,*) 'First compute eigenmodes with N command'
       GO TO 810
C
 61    CONTINUE
       IF(INDEX('P',CHPLOT(1:1)) .EQ. 0) THEN
C------ make root map plot first
        XORG0 = 0.0
        YORG0 = 0.0
        TMIN = 0.
        TMAX = 0.
        FMIN = 0.
        FMAX = 0.
        CALL PLEMAP(XORG0,YORG0, 
     &              IRUN1,IRUN2,
     &              KEVIEW,IRVIEW,
     &              LPROOT,LPRNUM,OVERLAY,
     &              TMIN,TMAX,TDEL,TFAC,
     &              FMIN,FMAX,FDEL,FFAC )
       ENDIF
C
       WRITE(*,*)
       WRITE(*,*) 'Click on root of eigenmode to be viewed...'
C
       CALL GETCURSORXY(XC1,YC1,CHKEY)
       XE1 = (XC1-XORG0)/TFAC + TMIN
       YE1 = (YC1-YORG0)/FFAC + FMIN
C
 64    CONTINUE
       IF(XE1 .LT. TMIN .OR. XE1 .GT. TMAX  .OR.
     &    YE1 .LT. FMIN .OR. YE1 .GT. FMAX       ) THEN
C------ click was outside of grid... manual input
 65     RINPUT(1) = 0.
        RINPUT(2) = 0.
        WRITE(*,1165) RINPUT(1), RINPUT(2)
 1165   FORMAT(/' Enter  sigma,omega  of root to be viewed:', 2F12.6)
        CALL READR(2,RINPUT,ERROR)
        IF(ERROR) GO TO 65
C
        ZCROOT = CMPLX( RINPUT(1) , RINPUT(2) )
C
       ELSE
C------ set root from its symbol-plot coordinates
        ZCROOT = CMPLX( XE1 , YE1 )
C
       ENDIF
C
C----- search current roots for one nearest to specified location
       IR = 0
       KEIG = 0
       DIST = 1.0E32
       DO JR = IRUN1, IRUN2
         DO K = 1, NEIGEN(JR)
           DIST1 = ABS( ZCROOT - EVAL(K,JR) )
           IF(DIST1.LE.DIST) THEN
            IR = JR
            KEIG = K
            DIST = DIST1
           ENDIF
         ENDDO
       ENDDO
C
       IF(IR.EQ.0 .OR. KEIG.EQ.0) THEN
        WRITE(*,*) 'Nearest root not identified'
        GO TO 810
       ENDIF
C
       CALL PLOTMD(AZIMOB, ELEVOB, TILTOB, ROBINV, KEIG, IR)
C
       KEVIEW = KEIG
       IRVIEW = IR
C
       CHPLOT = COMAND(1:2)
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
C---- Change time integration parameters
      ELSEIF(COMAND.EQ.'T   ') THEN
 70     WRITE(*,2070) DTIMED, DTMOVIE, TMOVIE
 2070   FORMAT(/' ==========================='
     &         /'  I ntegration  delta(t): ', G11.5,
     &         /'  O utput-frame delta(t): ', G11.5,
     &         /'  M aximum movie time   : ', G11.5 )
C
 72     CALL ASKC(' Select item,value^',ITEMC,COMARG)
 2072   FORMAT(' Enter new ',A,': ', $)
C
        IF(ITEMC.EQ.'    ') THEN
          GO TO 810
        ENDIF
C
        NINP = 1
        CALL GETFLT(COMARG,RINP,NINP,ERROR)
C
        IF    (INDEX('Ii',ITEMC(1:1)).NE.0) THEN
          IF(NINP.EQ.0) THEN
 74        WRITE(*,2072) 'integration delta(t)'
           READ (*,*,ERR=74) DTIMED
          ELSE
           DTIMED = RINP(1)
          ENDIF
C
        ELSEIF(INDEX('Oo',ITEMC(1:1)).NE.0) THEN
          IF(NINP.EQ.0) THEN
 75        WRITE(*,2072) 'output-frame delta(t)'
           READ (*,*,ERR=75) DTMOVIE
          ELSE
           DTMOVIE = RINP(1)
          ENDIF
C
        ELSEIF(INDEX('Mm',ITEMC(1:1)).NE.0) THEN
          IF(NINP.EQ.0) THEN
 76        WRITE(*,2072) 'maximum movie time'
           READ (*,*,ERR=76) TMOVIE
          ELSE
           TMOVIE = RINP(1)
          ENDIF
C
        ELSE
          WRITE(*,*) 'Item not recognized'
          GO TO 70
C
        ENDIF
        GO TO 70
C
C-----------------------------------------------------------------------
      ELSEIF(COMAND.EQ.'G   ') THEN
       LSVMOV = .NOT. LSVMOV
C
C-------------------------------------------------------------------
C---- write system matrices
      ELSEIF(COMAND.EQ.'S   ' .OR.
     &       COMAND.EQ.'SC  '      ) THEN
        DO 80 IR = IRUN1, IRUN2
          WRITE(*,2080) IR
 2080     FORMAT(/' Run case', I3,'...')
C
          CALL RUNCHK(IR,LOK)
          IF(.NOT.LOK) THEN
           WRITE(*,*) '** Skipping ill-posed run case', IR
           GO TO 80
          ENDIF
C
          ALFA    = PARVAL(IPALFA,IR)*DTR
          BETA    = PARVAL(IPBETA,IR)*DTR
          WROT(1) = PARVAL(IPROTX,IR)*2.0/BREF
          WROT(2) = PARVAL(IPROTY,IR)*2.0/CREF
          WROT(3) = PARVAL(IPROTZ,IR)*2.0/BREF
C
          XYZREF(1) = PARVAL(IPXCG,IR)
          XYZREF(2) = PARVAL(IPYCG,IR)
          XYZREF(3) = PARVAL(IPZCG,IR)
          CDREF     = PARVAL(IPCD0,IR)
C
          NITER = 10
          INFO = 1
          CALL EXEC(NITER,INFO,IR)
C
          IF(COMAND.EQ.'S   ') THEN
           CALL SYSMAT(IR,ASYS,BSYS,RSYS,NSYS)
          ELSE
           CALL APPMAT(IR,ASYS,BSYS,RSYS,NSYS)
          ENDIF
C
          LU = 6
          CALL SYSSHO(LU,ASYS,BSYS,RSYS,NSYS)
C
          WRITE(*,2082)
 2082     FORMAT(/' Enter output filename (or <Return>): ', $)
          READ(*,1000) FNSYS
          CALL STRIP(FNSYS,NFS)
          IF(NFS.EQ.0) GO TO 80
C
          OPEN(LUSYS,FILE=FNSYS,STATUS='OLD',ERR=81)
          IF(LOWRIT(FNSYS)) THEN
           REWIND(LUSYS)
           GO TO 82
          ELSE
           CLOSE(LUSYS)
           GO TO 80
          ENDIF
C
 81       OPEN(LUSYS,FILE=FNSYS,STATUS='NEW',ERR=85)
 82       CALL SYSSHO(LUSYS,ASYS,BSYS,RSYS,NSYS)
          CLOSE(LUSYS)
          GO TO 800
C
 85       CONTINUE
          WRITE(*,*) '* File OPEN error'
C
 80     CONTINUE
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
      END ! MODE


      SUBROUTINE EIGLST(LU,IR)
      INCLUDE 'AVL.INC'
C
      WRITE(LU,3100) IR, RTITLE(IR)
C
      DO J = 1, NEIGEN(IR)
      WRITE(LU,3200) J, REAL(EVAL(J,IR)), IMAG(EVAL(J,IR))
      WRITE(LU,3300)
     & 'u  ', REAL(EVEC(JEU,J,IR)), IMAG(EVEC(JEU,J,IR)),
     & 'v  ', REAL(EVEC(JEV,J,IR)), IMAG(EVEC(JEV,J,IR)),
     & 'x  ', REAL(EVEC(JEX,J,IR)), IMAG(EVEC(JEX,J,IR))
      WRITE(LU,3300)
     & 'w  ', REAL(EVEC(JEW,J,IR)), IMAG(EVEC(JEW,J,IR)),
     & 'p  ', REAL(EVEC(JEP,J,IR)), IMAG(EVEC(JEP,J,IR)),
     & 'y  ', REAL(EVEC(JEY,J,IR)), IMAG(EVEC(JEY,J,IR))
      WRITE(LU,3300)
     & 'q  ', REAL(EVEC(JEQ,J,IR)), IMAG(EVEC(JEQ,J,IR)),
     & 'r  ', REAL(EVEC(JER,J,IR)), IMAG(EVEC(JER,J,IR)),
     & 'z  ', REAL(EVEC(JEZ,J,IR)), IMAG(EVEC(JEZ,J,IR))
      WRITE(LU,3300)
     & 'the', REAL(EVEC(JETH,J,IR)), IMAG(EVEC(JETH,J,IR)),
     & 'phi', REAL(EVEC(JEPH,J,IR)), IMAG(EVEC(JEPH,J,IR)),
     & 'psi', REAL(EVEC(JEPS,J,IR)), IMAG(EVEC(JEPS,J,IR))
      ENDDO

 3100 FORMAT(/1X,'Run case',I3,':  ',A)
 3200 FORMAT(/1X,' mode',I2,':', 2G14.6)
 3300 FORMAT( 1X, A,':', 2F11.4, 6X, A,':', 2F11.4, 6X, A,':', 2G12.4)
C
      RETURN
      END ! EIGLST



      SUBROUTINE EIGOUT(FILNAM, IRUN1,IRUN2, SAVED)
      INCLUDE 'AVL.INC'
C
      CHARACTER*(*) FILNAM
      LOGICAL SAVED
C
      LOGICAL LOWRIT
C
      LU = 2
      OPEN(LU,FILE=FILNAM,STATUS='OLD',ERR=17)
      IF(LOWRIT(FILNAM)) THEN
       GO TO 18
      ELSE
       CLOSE(LU)
       WRITE(*,*) 'Eigenvalues not saved.'
       SAVED = .FALSE.
       RETURN
      ENDIF
C
 17   OPEN(LU,FILE=FILNAM,STATUS='UNKNOWN',ERR=80)
 18   CONTINUE
      REWIND(LU)
C
C---- write header
      WRITE(LU,1100) TITLE
 1100 FORMAT('# ', A
     &      /'# '
     &      /'#   Run case     Eigenvalue')
C
 2300 FORMAT(1X, I7, 2G18.8, f12.4 )
C
      DO IR = IRUN1, IRUN2
        DO KEIG = 1, NEIGEN(IR)
          WRITE(LU,2300) IR,
     &                 REAL(EVAL(KEIG,IR)),
     &                 IMAG(EVAL(KEIG,IR))
        ENDDO
      ENDDO
C
      CLOSE(LU)
      SAVED = .TRUE.
      RETURN
C
 80   CONTINUE
      SAVED = .FALSE.
      RETURN
      END ! EIGOUT




      SUBROUTINE EIGINP(FILNAM, ERROR)
      INCLUDE 'AVL.INC'
C
      CHARACTER*(*) FILNAM
      LOGICAL ERROR
C
      CHARACTER*1 DUMMY
C
C---- assume that read will fail
      ERROR = .TRUE.
C
 1000 FORMAT(A)
C
      DO IR = 1, NRMAX
        NEIGENDAT(IR) = 0
      ENDDO
C
      LU = 2
      OPEN(LU,FILE=FILNAM,STATUS='OLD',ERR=85)
      REWIND(LU)
C
C---- read header
      READ(LU,1000,ERR=90) DUMMY
      READ(LU,1000,ERR=90) DUMMY
      READ(LU,1000,ERR=90) DUMMY
C
C---- find index of forcing parameter of this file
      DO ILINE = 1, 123456
 15     READ(LU,*,END=20,ERR=90) IR, SIGMA, OMEGA
        IF(IR.LT.1 .OR. IR.GT.NRMAX) GO TO 15
C
        KEIG = NEIGENDAT(IR) + 1
        IF(KEIG .LE. JEMAX) THEN
         EVALDAT(KEIG,IR) = CMPLX(SIGMA,OMEGA)
         NEIGENDAT(IR) = KEIG
        ENDIF
      ENDDO
C
 20   CONTINUE
      CLOSE(LU)
      ERROR = .FALSE.
      RETURN
C
 85   CONTINUE
      WRITE(*,*) 'File OPEN error'
      RETURN
C
 90   CONTINUE
      WRITE(*,*) 'File READ error'
      CLOSE(LU)
      RETURN
      END ! EIGINP





      SUBROUTINE RUNCHK(JRUN,OK)
C------------------------------------------------------------------
C     Returns OK=t if run case JRUN has no redundant constraints.
C------------------------------------------------------------------
      INCLUDE 'AVL.INC'
      LOGICAL OK
C
      OK = .TRUE.
      DO IV = 1, NVTOT
        DO JV = 1, NVTOT
          IF(IV.NE.JV .AND. ICON(IV,JRUN).EQ.ICON(JV,JRUN)) THEN
           OK = .FALSE.
          ENDIF
        ENDDO
      ENDDO
C
      RETURN
      END ! RUNCHK



      SUBROUTINE SYSMAT(IR,ASYS,BSYS,RSYS,NSYS)
C------------------------------------------------------------------
C     Computes system matrices for run case IR.
C     Current forces and derivatives are assumed to be correct.
C------------------------------------------------------------------
      INCLUDE 'AVL.INC'
      REAL*8 ASYS(JEMAX,JEMAX),BSYS(JEMAX,NDMAX),RSYS(JEMAX)
C
C
      LOGICAL LTERR
C
      REAL RINER(3,3),
     &     MAMAT(3,3),
     &     MAINV(3,3),
     &     RIMAT(3,3),
     &     RIINV(3,3),
     &     P(3), P_U(3,NUMAX),
     &     H(3), H_U(3,NUMAX),
     &     WXP(3), WXP_U(3,NUMAX),
     &     WXH(3), WXH_U(3,NUMAX),
     &     MIF(3), MIF_U(3,NUMAX), MIF_D(3,NDMAX),
     &     RIM(3), RIM_U(3,NUMAX), RIM_D(3,NDMAX),
     &     PRF(3), PRF_U(3,NUMAX),
     &     PRM(3), PRM_U(3,NUMAX)
C
      REAL ANG(3), TT(3,3), TT_ANG(3,3,3), RT(3,3), RT_ANG(3,3,3)
C
      INTEGER ICRS(3), JCRS(3)
      DATA ICRS / 2, 3, 1 / , JCRS / 3, 1, 2 /
C
C
C
      GEE = PARVAL(IPGEE,IR)
      RHO = PARVAL(IPRHO,IR)
      VEE = PARVAL(IPVEE,IR)
      PHI = PARVAL(IPPHI,IR)
      THE = PARVAL(IPTHE,IR)
      PSI = PARVAL(IPPSI,IR)
      XCG = PARVAL(IPXCG,IR)
      YCG = PARVAL(IPYCG,IR)
      ZCG = PARVAL(IPZCG,IR)
      RMASS = PARVAL(IPMASS,IR)
      RINER(1,1) = PARVAL(IPIXX,IR)
      RINER(2,2) = PARVAL(IPIYY,IR)
      RINER(3,3) = PARVAL(IPIZZ,IR)
      RINER(1,2) = PARVAL(IPIXY,IR)
      RINER(2,3) = PARVAL(IPIYZ,IR)
      RINER(3,1) = PARVAL(IPIZX,IR)
      RINER(2,1) = PARVAL(IPIXY,IR)
      RINER(3,2) = PARVAL(IPIYZ,IR)
      RINER(1,3) = PARVAL(IPIZX,IR)
C
      DCL_U = PARVAL(IPCLU,IR)
      DCM_U = PARVAL(IPCMU,IR)
      DCL_A = PARVAL(IPCLA,IR)
      DCM_A = PARVAL(IPCMA,IR)
C
      SINA = SIN(PARVAL(IPALFA,IR))
      COSA = COS(PARVAL(IPALFA,IR))
C
      LTERR = .FALSE.
      IF(VEE .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Velocity.  Specify with run file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RMASS .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Mass.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RINER(1,1) .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Ixx.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RINER(2,2) .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Iyy.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RINER(3,3) .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Izz.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
C
      IF(LTERR) THEN
       WRITE(*,*)
       WRITE(*,*) 'Eigenmodes not computed for run case', IR
       RETURN
      ENDIF
C
      SREFD = SREF*UNITL**2
      BREFD = BREF*UNITL
      CREFD = CREF*UNITL
C
      XYZREF(1) = XCG
      XYZREF(2) = YCG
      XYZREF(3) = ZCG
C
      QS  = 0.5*RHO*VEE**2 * SREFD
      QSM = QS/RMASS
C
      ROT = VEE/UNITL
C
C---- set mass and inertia tensors (real + apparent)
      DO K = 1, 3
        MAMAT(K,1) = AMASS(K,1)*RHO
        MAMAT(K,2) = AMASS(K,2)*RHO
        MAMAT(K,3) = AMASS(K,3)*RHO
        MAMAT(K,K) = MAMAT(K,K) + RMASS
C
        RIMAT(K,1) = RINER(K,1) + AINER(K,1)*RHO
        RIMAT(K,2) = RINER(K,2) + AINER(K,2)*RHO
        RIMAT(K,3) = RINER(K,3) + AINER(K,3)*RHO
      ENDDO
C
C---- invert mass and inertia tensors
      CALL M3INV(MAMAT,MAINV)
      CALL M3INV(RIMAT,RIINV)
C
C                         -   = -                        -   = -
C---- set linear momentum P = m V  and angular momentum  H = I W, in AVL axes
      DO K = 1, 3
        P(K) = -(  MAMAT(K,1)*VINF(1)
     &           + MAMAT(K,2)*VINF(2)
     &           + MAMAT(K,3)*VINF(3) )*VEE
        P_U(K,1) = -MAMAT(K,1)*VEE
        P_U(K,2) = -MAMAT(K,2)*VEE
        P_U(K,3) = -MAMAT(K,3)*VEE
        P_U(K,4) = 0.
        P_U(K,5) = 0.
        P_U(K,6) = 0.
C
        H(K) = (  RIMAT(K,1)*WROT(1)
     &          + RIMAT(K,2)*WROT(2)
     &          + RIMAT(K,3)*WROT(3) )*ROT
        H_U(K,1) = 0.
        H_U(K,2) = 0.
        H_U(K,3) = 0.
        H_U(K,4) = RIMAT(K,1)*ROT
        H_U(K,5) = RIMAT(K,2)*ROT
        H_U(K,6) = RIMAT(K,3)*ROT
      ENDDO
C
C---- set linear and angular momentum rates  WXP = W x P,   WXH = W x H
      DO K = 1, 3
        I = ICRS(K)
        J = JCRS(K)
        WXP(K) = (WROT(I)*P(J) - WROT(J)*P(I))*ROT
        WXH(K) = (WROT(I)*H(J) - WROT(J)*H(I))*ROT
C
        WXP_U(K,1) = (WROT(I)*P_U(J,1) - WROT(J)*P_U(I,1))*ROT
        WXP_U(K,2) = (WROT(I)*P_U(J,2) - WROT(J)*P_U(I,2))*ROT
        WXP_U(K,3) = (WROT(I)*P_U(J,3) - WROT(J)*P_U(I,3))*ROT
        WXP_U(K,4) = 0.
        WXP_U(K,5) = 0.
        WXP_U(K,6) = 0.
        WXP_U(K,I+3) = WXP_U(K,I+3) + P(J)*ROT
        WXP_U(K,J+3) = WXP_U(K,J+3) - P(I)*ROT
C
        WXH_U(K,1) = 0.
        WXH_U(K,2) = 0.
        WXH_U(K,3) = 0.
        WXH_U(K,4) = (WROT(I)*H_U(J,4) - WROT(J)*H_U(I,4))*ROT
        WXH_U(K,5) = (WROT(I)*H_U(J,5) - WROT(J)*H_U(I,5))*ROT
        WXH_U(K,6) = (WROT(I)*H_U(J,6) - WROT(J)*H_U(I,6))*ROT
        WXH_U(K,I+3) = WXH_U(K,I+3) + H(J)*ROT
        WXH_U(K,J+3) = WXH_U(K,J+3) - H(I)*ROT
      ENDDO
C
C          =    -   =    -   =     - -    =     - -
C---- set  m^-1 F,  I^-1 M,  m^-1 (WxP),  I^-1 (WxH)
      DO K = 1, 3
        MIF(K) = MAINV(K,1)*CXTOT*QS
     &         + MAINV(K,2)*CYTOT*QS
     &         + MAINV(K,3)*CZTOT*QS
        RIM(K) = RIINV(K,1)*CRTOT*QS*BREFD
     &         + RIINV(K,2)*CMTOT*QS*CREFD
     &         + RIINV(K,3)*CNTOT*QS*BREFD
        PRF(K) = MAINV(K,1)*WXP(1)
     &         + MAINV(K,2)*WXP(2)
     &         + MAINV(K,3)*WXP(3)
        PRM(K) = RIINV(K,1)*WXH(1)
     &         + RIINV(K,2)*WXH(2)
     &         + RIINV(K,3)*WXH(3)
        DO IU = 1, 6
          MIF_U(K,IU) = 
     &           MAINV(K,1)*CXTOT_U(IU)*QS
     &         + MAINV(K,2)*CYTOT_U(IU)*QS
     &         + MAINV(K,3)*CZTOT_U(IU)*QS
          RIM_U(K,IU) = 
     &           RIINV(K,1)*CRTOT_U(IU)*QS*BREFD
     &         + RIINV(K,2)*CMTOT_U(IU)*QS*CREFD
     &         + RIINV(K,3)*CNTOT_U(IU)*QS*BREFD
          PRF_U(K,IU) =
     &           MAINV(K,1)*WXP_U(1,IU)
     &         + MAINV(K,2)*WXP_U(2,IU)
     &         + MAINV(K,3)*WXP_U(3,IU)
          PRM_U(K,IU) =
     &           RIINV(K,1)*WXH_U(1,IU)
     &         + RIINV(K,2)*WXH_U(2,IU)
     &         + RIINV(K,3)*WXH_U(3,IU)
        ENDDO

        DO N = 1, NCONTROL
          MIF_D(K,N) = 
     &           MAINV(K,1)*CXTOT_D(N)*QS
     &         + MAINV(K,2)*CYTOT_D(N)*QS
     &         + MAINV(K,3)*CZTOT_D(N)*QS
          RIM_D(K,N) = 
     &           RIINV(K,1)*CRTOT_D(N)*QS*BREFD
     &         + RIINV(K,2)*CMTOT_D(N)*QS*CREFD
     &         + RIINV(K,3)*CNTOT_D(N)*QS*BREFD
        ENDDO
C
C------ add additional derivatives, from viscous or Mach effects, or whatever
        IU = 1
        MIF_U(K,IU) = MIF_U(K,IU)  -  MAINV(K,3)*DCL_U*QS
        RIM_U(K,IU) = RIM_U(K,IU)  -  RIINV(K,2)*DCM_U*QS*CREFD
C
        IU = 3
        MIF_U(K,IU) = MIF_U(K,IU)  +  MAINV(K,3)*DCL_A*QS
        RIM_U(K,IU) = RIM_U(K,IU)  +  RIINV(K,2)*DCM_A*QS*CREFD
C
      ENDDO
C
      ANG(1) = PHI*DTR
      ANG(2) = THE*DTR
      ANG(3) = PSI*DTR
      CALL ROTENS3(ANG,TT,TT_ANG)
      CALL RATEKI3(ANG,RT,RT_ANG)
C
      NSYS = 12
      DO IEQ = 1, NSYS
        DO JE = 1, NSYS
          ASYS(IEQ,JE) = 0.
        ENDDO
        DO N = 1, NCONTROL
          BSYS(IEQ,N) = 0.
        ENDDO
      ENDDO
C
C---- x-acceleration
      IEQ = JEU
      K = 1
      RSYS(IEQ)     =   MIF(K)     - PRF(K)      - GEE*TT(3,K)
      ASYS(IEQ,JEU) = -(MIF_U(K,1) - PRF_U(K,1)) / VEE
      ASYS(IEQ,JEV) = -(MIF_U(K,2) - PRF_U(K,2)) / VEE
      ASYS(IEQ,JEW) = -(MIF_U(K,3) - PRF_U(K,3)) / VEE
      ASYS(IEQ,JEP) =  (MIF_U(K,4) - PRF_U(K,4)) / ROT
      ASYS(IEQ,JEQ) =  (MIF_U(K,5) - PRF_U(K,5)) / ROT
      ASYS(IEQ,JER) =  (MIF_U(K,6) - PRF_U(K,6)) / ROT
      ASYS(IEQ,JEPH)=                            - GEE*TT_ANG(3,K,1)
      ASYS(IEQ,JETH)=                            - GEE*TT_ANG(3,K,2)
      ASYS(IEQ,JEPS)=                            - GEE*TT_ANG(3,K,3)
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =   MIF_D(K,N)
      ENDDO

c      MIF_U(1,2)
c      write(*,*)
c      write(*,*) '* 1u', MIF_U(K,2), PRF_U(K,2)

C
C---- y-acceleration
      IEQ = JEV
      K = 2
      RSYS(IEQ)     =   MIF(K)     - PRF(K)      - GEE*TT(3,K)
      ASYS(IEQ,JEU) = -(MIF_U(K,1) - PRF_U(K,1)) / VEE
      ASYS(IEQ,JEV) = -(MIF_U(K,2) - PRF_U(K,2)) / VEE
      ASYS(IEQ,JEW) = -(MIF_U(K,3) - PRF_U(K,3)) / VEE
      ASYS(IEQ,JEP) =  (MIF_U(K,4) - PRF_U(K,4)) / ROT
      ASYS(IEQ,JEQ) =  (MIF_U(K,5) - PRF_U(K,5)) / ROT
      ASYS(IEQ,JER) =  (MIF_U(K,6) - PRF_U(K,6)) / ROT
      ASYS(IEQ,JEPH)=                            - GEE*TT_ANG(3,K,1)
      ASYS(IEQ,JETH)=                            - GEE*TT_ANG(3,K,2)
      ASYS(IEQ,JEPS)=                            - GEE*TT_ANG(3,K,3)
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =   MIF_D(K,N)
      ENDDO
C
C---- z-acceleration
      IEQ = JEW
      K = 3
      RSYS(IEQ)     =   MIF(K)     - PRF(K)      - GEE*TT(3,K)
      ASYS(IEQ,JEU) = -(MIF_U(K,1) - PRF_U(K,1)) / VEE
      ASYS(IEQ,JEV) = -(MIF_U(K,2) - PRF_U(K,2)) / VEE
      ASYS(IEQ,JEW) = -(MIF_U(K,3) - PRF_U(K,3)) / VEE
      ASYS(IEQ,JEP) =  (MIF_U(K,4) - PRF_U(K,4)) / ROT
      ASYS(IEQ,JEQ) =  (MIF_U(K,5) - PRF_U(K,5)) / ROT
      ASYS(IEQ,JER) =  (MIF_U(K,6) - PRF_U(K,6)) / ROT
      ASYS(IEQ,JEPH)=                            - GEE*TT_ANG(3,K,1)
      ASYS(IEQ,JETH)=                            - GEE*TT_ANG(3,K,2)
      ASYS(IEQ,JEPS)=                            - GEE*TT_ANG(3,K,3)
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =   MIF_D(K,N)
      ENDDO
C
C---- x-ang.accel.
      IEQ = JEP
      K = 1
      RSYS(IEQ)     =   RIM(K)     - PRM(K)
      ASYS(IEQ,JEU) = -(RIM_U(K,1) - PRM_U(K,1)) / VEE
      ASYS(IEQ,JEV) = -(RIM_U(K,2) - PRM_U(K,2)) / VEE
      ASYS(IEQ,JEW) = -(RIM_U(K,3) - PRM_U(K,3)) / VEE
      ASYS(IEQ,JEP) =  (RIM_U(K,4) - PRM_U(K,4)) / ROT
      ASYS(IEQ,JEQ) =  (RIM_U(K,5) - PRM_U(K,5)) / ROT
      ASYS(IEQ,JER) =  (RIM_U(K,6) - PRM_U(K,6)) / ROT
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =   RIM_D(K,N)
      ENDDO
C
C---- y-ang.accel.
      IEQ = JEQ
      K = 2
      RSYS(IEQ)     =   RIM(K)     - PRM(K)
      ASYS(IEQ,JEU) = -(RIM_U(K,1) - PRM_U(K,1)) / VEE
      ASYS(IEQ,JEV) = -(RIM_U(K,2) - PRM_U(K,2)) / VEE
      ASYS(IEQ,JEW) = -(RIM_U(K,3) - PRM_U(K,3)) / VEE
      ASYS(IEQ,JEP) =  (RIM_U(K,4) - PRM_U(K,4)) / ROT
      ASYS(IEQ,JEQ) =  (RIM_U(K,5) - PRM_U(K,5)) / ROT
      ASYS(IEQ,JER) =  (RIM_U(K,6) - PRM_U(K,6)) / ROT
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =   RIM_D(K,N)
      ENDDO
C
C---- z-ang.accel.
      IEQ = JER
      K = 3
      RSYS(IEQ)     =   RIM(K)     - PRM(K)
      ASYS(IEQ,JEU) = -(RIM_U(K,1) - PRM_U(K,1)) / VEE
      ASYS(IEQ,JEV) = -(RIM_U(K,2) - PRM_U(K,2)) / VEE
      ASYS(IEQ,JEW) = -(RIM_U(K,3) - PRM_U(K,3)) / VEE
      ASYS(IEQ,JEP) =  (RIM_U(K,4) - PRM_U(K,4)) / ROT
      ASYS(IEQ,JEQ) =  (RIM_U(K,5) - PRM_U(K,5)) / ROT
      ASYS(IEQ,JER) =  (RIM_U(K,6) - PRM_U(K,6)) / ROT
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =   RIM_D(K,N)
      ENDDO
C
C---- phi rate
      IEQ = JEPH
      K = 1
      RSYS(IEQ)     = ROT*(  RT(K,1)*WROT(1)
     &                     + RT(K,2)*WROT(2)
     &                     + RT(K,3)*WROT(3) )
      ASYS(IEQ,JEP) =        RT(K,1)
      ASYS(IEQ,JEQ) =        RT(K,2)
      ASYS(IEQ,JER) =        RT(K,3)
      ASYS(IEQ,JEPH)= ROT*(  RT_ANG(K,1,1)*WROT(1)
     &                     + RT_ANG(K,2,1)*WROT(2)
     &                     + RT_ANG(K,3,1)*WROT(3) )
      ASYS(IEQ,JETH)= ROT*(  RT_ANG(K,1,2)*WROT(1)
     &                     + RT_ANG(K,2,2)*WROT(2)
     &                     + RT_ANG(K,3,2)*WROT(3) )
      ASYS(IEQ,JEPS)= ROT*(  RT_ANG(K,1,3)*WROT(1)
     &                     + RT_ANG(K,2,3)*WROT(2)
     &                     + RT_ANG(K,3,3)*WROT(3) )
C
C---- theta rate
      IEQ = JETH
      K = 2
      RSYS(IEQ)     = ROT*(  RT(K,1)*WROT(1)
     &                     + RT(K,2)*WROT(2)
     &                     + RT(K,3)*WROT(3) )
      ASYS(IEQ,JEP) =        RT(K,1)
      ASYS(IEQ,JEQ) =        RT(K,2)
      ASYS(IEQ,JER) =        RT(K,3)
      ASYS(IEQ,JEPH)= ROT*(  RT_ANG(K,1,1)*WROT(1)
     &                     + RT_ANG(K,2,1)*WROT(2)
     &                     + RT_ANG(K,3,1)*WROT(3) )
      ASYS(IEQ,JETH)= ROT*(  RT_ANG(K,1,2)*WROT(1)
     &                     + RT_ANG(K,2,2)*WROT(2)
     &                     + RT_ANG(K,3,2)*WROT(3) )
      ASYS(IEQ,JEPS)= ROT*(  RT_ANG(K,1,3)*WROT(1)
     &                     + RT_ANG(K,2,3)*WROT(2)
     &                     + RT_ANG(K,3,3)*WROT(3) )
C
C---- psi rate
      IEQ = JEPS
      K = 3
      RSYS(IEQ)     = ROT*(  RT(K,1)*WROT(1)
     &                     + RT(K,2)*WROT(2)
     &                     + RT(K,3)*WROT(3) )
      ASYS(IEQ,JEP) =        RT(K,1)
      ASYS(IEQ,JEQ) =        RT(K,2)
      ASYS(IEQ,JER) =        RT(K,3)
      ASYS(IEQ,JEPH)= ROT*(  RT_ANG(K,1,1)*WROT(1)
     &                     + RT_ANG(K,2,1)*WROT(2)
     &                     + RT_ANG(K,3,1)*WROT(3) )
      ASYS(IEQ,JETH)= ROT*(  RT_ANG(K,1,2)*WROT(1)
     &                     + RT_ANG(K,2,2)*WROT(2)
     &                     + RT_ANG(K,3,2)*WROT(3) )
      ASYS(IEQ,JEPS)= ROT*(  RT_ANG(K,1,3)*WROT(1)
     &                     + RT_ANG(K,2,3)*WROT(2)
     &                     + RT_ANG(K,3,3)*WROT(3) )
C
C---- x-velocity
      IEQ = JEX
      K = 1
      RSYS(IEQ)     = -(  TT(K,1)*VINF(1)
     &                  + TT(K,2)*VINF(2)
     &                  + TT(K,3)*VINF(3) )*VEE
      ASYS(IEQ,JEU) =     TT(K,1)
      ASYS(IEQ,JEV) =     TT(K,2)
      ASYS(IEQ,JEW) =     TT(K,3)
      ASYS(IEQ,JEPH)= -(  TT_ANG(K,1,1)*VINF(1)
     &                  + TT_ANG(K,2,1)*VINF(2)
     &                  + TT_ANG(K,3,1)*VINF(3) )*VEE
      ASYS(IEQ,JETH)= -(  TT_ANG(K,1,2)*VINF(1)
     &                  + TT_ANG(K,2,2)*VINF(2)
     &                  + TT_ANG(K,3,2)*VINF(3) )*VEE
      ASYS(IEQ,JEPS)= -(  TT_ANG(K,1,3)*VINF(1)
     &                  + TT_ANG(K,2,3)*VINF(2)
     &                  + TT_ANG(K,3,3)*VINF(3) )*VEE
C
C---- y-velocity
      IEQ = JEY
      K = 2
      RSYS(IEQ)     = -(  TT(K,1)*VINF(1)
     &                  + TT(K,2)*VINF(2)
     &                  + TT(K,3)*VINF(3) )*VEE
      ASYS(IEQ,JEU) =     TT(K,1)
      ASYS(IEQ,JEV) =     TT(K,2)
      ASYS(IEQ,JEW) =     TT(K,3)
      ASYS(IEQ,JEPH)= -(  TT_ANG(K,1,1)*VINF(1)
     &                  + TT_ANG(K,2,1)*VINF(2)
     &                  + TT_ANG(K,3,1)*VINF(3) )*VEE
      ASYS(IEQ,JETH)= -(  TT_ANG(K,1,2)*VINF(1)
     &                  + TT_ANG(K,2,2)*VINF(2)
     &                  + TT_ANG(K,3,2)*VINF(3) )*VEE
      ASYS(IEQ,JEPS)= -(  TT_ANG(K,1,3)*VINF(1)
     &                  + TT_ANG(K,2,3)*VINF(2)
     &                  + TT_ANG(K,3,3)*VINF(3) )*VEE
C
C---- z-velocity
      IEQ = JEZ
      K = 3
      RSYS(IEQ)     = -(  TT(K,1)*VINF(1)
     &                  + TT(K,2)*VINF(2)
     &                  + TT(K,3)*VINF(3) )*VEE
      ASYS(IEQ,JEU) =     TT(K,1)
      ASYS(IEQ,JEV) =     TT(K,2)
      ASYS(IEQ,JEW) =     TT(K,3)
      ASYS(IEQ,JEPH)= -(  TT_ANG(K,1,1)*VINF(1)
     &                  + TT_ANG(K,2,1)*VINF(2)
     &                  + TT_ANG(K,3,1)*VINF(3) )*VEE
      ASYS(IEQ,JETH)= -(  TT_ANG(K,1,2)*VINF(1)
     &                  + TT_ANG(K,2,2)*VINF(2)
     &                  + TT_ANG(K,3,2)*VINF(3) )*VEE
      ASYS(IEQ,JEPS)= -(  TT_ANG(K,1,3)*VINF(1)
     &                  + TT_ANG(K,2,3)*VINF(2)
     &                  + TT_ANG(K,3,3)*VINF(3) )*VEE
C
c      write(*,*) 'H  ', H
c      write(*,*) 'WxH', WXH
c      write(*,*) 'I-1 M  ', RIM
c      write(*,*) 'I-1 WxH', PRM
C
c      write(*,*) nzmax, nsys
C
      RETURN
      END ! SYSMAT




      SUBROUTINE APPMAT(IR,ASYS,BSYS,RSYS,NSYS)
C------------------------------------------------------------------
C     Computes system matrices for run case IR,
C     using the approximate expression in Etkin.
C
C     Current forces and derivatives are assumed to be correct.
C------------------------------------------------------------------
      INCLUDE 'AVL.INC'
      REAL*8 ASYS(JEMAX,JEMAX),BSYS(JEMAX,NDMAX),RSYS(JEMAX)
C
      LOGICAL LTERR
C
      REAL RINER(3,3),
     &     MAMAT(3,3),
     &     MAINV(3,3),
     &     RIMAT(3,3),
     &     RIINV(3,3),
     &     P(3), P_U(3,NUMAX),
     &     H(3), H_U(3,NUMAX),
     &     WXP(3), WXP_U(3,NUMAX),
     &     WXH(3), WXH_U(3,NUMAX),
     &     MIF(3), MIF_U(3,NUMAX), MIF_D(3,NDMAX),
     &     RIM(3), RIM_U(3,NUMAX), RIM_D(3,NDMAX),
     &     PRF(3), PRF_U(3,NUMAX),
     &     PRM(3), PRM_U(3,NUMAX)
C
      REAL ANG(3), TT(3,3), TT_ANG(3,3,3), RT(3,3), RT_ANG(3,3,3)
C
      INTEGER ICRS(3), JCRS(3)
      DATA ICRS / 2, 3, 1 / , JCRS / 3, 1, 2 /
C
C
C
      GEE = PARVAL(IPGEE,IR)
      RHO = PARVAL(IPRHO,IR)
      VEE = PARVAL(IPVEE,IR)
      PHI = PARVAL(IPPHI,IR)
      THE = PARVAL(IPTHE,IR)
      PSI = PARVAL(IPPSI,IR)
      XCG = PARVAL(IPXCG,IR)
      YCG = PARVAL(IPYCG,IR)
      ZCG = PARVAL(IPZCG,IR)
      RMASS = PARVAL(IPMASS,IR)
      RINER(1,1) = PARVAL(IPIXX,IR)
      RINER(2,2) = PARVAL(IPIYY,IR)
      RINER(3,3) = PARVAL(IPIZZ,IR)
      RINER(1,2) = PARVAL(IPIXY,IR)
      RINER(2,3) = PARVAL(IPIYZ,IR)
      RINER(3,1) = PARVAL(IPIZX,IR)
      RINER(2,1) = PARVAL(IPIXY,IR)
      RINER(3,2) = PARVAL(IPIYZ,IR)
      RINER(1,3) = PARVAL(IPIZX,IR)
C
      DCL_U = PARVAL(IPCLU,IR)
      DCM_U = PARVAL(IPCMU,IR)
      DCL_A = PARVAL(IPCLA,IR)
      DCM_A = PARVAL(IPCMA,IR)
C
      LTERR = .FALSE.
      IF(VEE .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Velocity.  Specify with run file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RMASS .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Mass.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RINER(1,1) .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Ixx.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RINER(2,2) .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Iyy.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
      IF(RINER(3,3) .LE. 0.0) THEN
       WRITE(*,*)
       WRITE(*,*) '** Zero Izz.  Specify with mass file or M menu'
       LTERR = .TRUE.
      ENDIF
C
      IF(LTERR) THEN
       WRITE(*,*)
       WRITE(*,*) 'Eigenmodes not computed for run case', IR
       RETURN
      ENDIF
C
      SREFD = SREF*UNITL**2
      BREFD = BREF*UNITL
      CREFD = CREF*UNITL
C
      XYZREF(1) = XCG
      XYZREF(2) = YCG
      XYZREF(3) = ZCG
C
      QS  = 0.5*RHO*VEE**2 * SREFD
      QSM = QS/RMASS
C
      QSC = QS*CREFD
      QSB = QS*BREFD
C
      RINXX = RINER(1,1)
      RINYY = RINER(2,2)
      RINZZ = RINER(3,3)
C
C
      ROT = VEE/UNITL
C
C
      ANG(1) = PHI*DTR
      ANG(2) = THE*DTR
      ANG(3) = PSI*DTR
      CALL ROTENS3(ANG,TT,TT_ANG)
      CALL RATEKI3(ANG,RT,RT_ANG)
C
      NSYS = 12
      DO IEQ = 1, NSYS
        DO JE = 1, NSYS
          ASYS(IEQ,JE) = 0.
        ENDDO
        DO N = 1, NCONTROL
          BSYS(IEQ,N) = 0.
        ENDDO
      ENDDO
C
C---- x-acceleration
      IEQ = JEU
      ASYS(IEQ,JEU) = -CXTOT_U(1)*QS /RMASS / VEE
      ASYS(IEQ,JEW) = -CXTOT_U(3)*QS /RMASS / VEE
      ASYS(IEQ,JEQ) =  CXTOT_U(5)*QS /RMASS / ROT  +  VINF(3)*VEE
      ASYS(IEQ,JETH)=  GEE
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =  CXTOT_D(N)*QS /RMASS
      ENDDO
C
C---- z-acceleration
      IEQ = JEW
      ASYS(IEQ,JEU) = -CZTOT_U(1)*QS /RMASS / VEE
      ASYS(IEQ,JEW) = -CZTOT_U(3)*QS /RMASS / VEE
      ASYS(IEQ,JEQ) =  CZTOT_U(5)*QS /RMASS / ROT  -  VINF(1)*VEE
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =  CZTOT_D(N)*QS /RMASS
      ENDDO
C
C---- y-ang.accel.
      IEQ = JEQ
      ASYS(IEQ,JEU) = -CMTOT_U(1)*QSC/RINYY / VEE
      ASYS(IEQ,JEW) = -CMTOT_U(3)*QSC/RINYY / VEE
      ASYS(IEQ,JEQ) =  CMTOT_U(5)*QSC/RINYY / ROT
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =  CMTOT_D(N)*QSC/RINYY
      ENDDO
C
C---- theta rate
      IEQ = JETH
      ASYS(IEQ,JEQ) =  1.0
C
C
C
C---- y-acceleration
      IEQ = JEV
      ASYS(IEQ,JEV) = -CYTOT_U(2)*QS /RMASS / VEE
      ASYS(IEQ,JEP) =  CYTOT_U(4)*QS /RMASS / ROT  -  VINF(3)*VEE
      ASYS(IEQ,JER) =  CYTOT_U(6)*QS /RMASS / ROT  +  VINF(1)*VEE
      ASYS(IEQ,JEPH)=  GEE
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =  CYTOT_D(N)*QS /RMASS
      ENDDO
C
C---- x-ang.accel.
      IEQ = JEP
      ASYS(IEQ,JEV) = -CRTOT_U(2)*QSB/RINXX / VEE
      ASYS(IEQ,JEP) =  CRTOT_U(4)*QSB/RINXX / ROT
      ASYS(IEQ,JER) =  CRTOT_U(6)*QSB/RINXX / ROT
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =  CRTOT_D(N)*QSB/RINXX
      ENDDO
C
C---- z-ang.accel.
      IEQ = JER
      ASYS(IEQ,JEV) = -CNTOT_U(2)*QSB/RINZZ / VEE
      ASYS(IEQ,JEP) =  CNTOT_U(4)*QSB/RINZZ / ROT
      ASYS(IEQ,JER) =  CNTOT_U(6)*QSB/RINZZ / ROT
      DO N = 1, NCONTROL
        BSYS(IEQ,N) =  CNTOT_D(N)*QSB/RINZZ
      ENDDO
C
C---- phi rate
      IEQ = JEPH
      ASYS(IEQ,JEP) =  -1.0



C---- psi rate
      IEQ = JEPS
      K = 3
      RSYS(IEQ)     = ROT*(  RT(K,1)*WROT(1)
     &                     + RT(K,2)*WROT(2)
     &                     + RT(K,3)*WROT(3) )
      ASYS(IEQ,JEP) =        RT(K,1)
      ASYS(IEQ,JEQ) =        RT(K,2)
      ASYS(IEQ,JER) =        RT(K,3)
      ASYS(IEQ,JEPH)= ROT*(  RT_ANG(K,1,1)*WROT(1)
     &                     + RT_ANG(K,2,1)*WROT(2)
     &                     + RT_ANG(K,3,1)*WROT(3) )
      ASYS(IEQ,JETH)= ROT*(  RT_ANG(K,1,2)*WROT(1)
     &                     + RT_ANG(K,2,2)*WROT(2)
     &                     + RT_ANG(K,3,2)*WROT(3) )
      ASYS(IEQ,JEPS)= ROT*(  RT_ANG(K,1,3)*WROT(1)
     &                     + RT_ANG(K,2,3)*WROT(2)
     &                     + RT_ANG(K,3,3)*WROT(3) )
C
C---- x-velocity
      IEQ = JEX
      K = 1
      RSYS(IEQ)     = -(  TT(K,1)*VINF(1)
     &                  + TT(K,2)*VINF(2)
     &                  + TT(K,3)*VINF(3) )*VEE
      ASYS(IEQ,JEU) =     TT(K,1)
      ASYS(IEQ,JEV) =     TT(K,2)
      ASYS(IEQ,JEW) =     TT(K,3)
      ASYS(IEQ,JEPH)= -(  TT_ANG(K,1,1)*VINF(1)
     &                  + TT_ANG(K,2,1)*VINF(2)
     &                  + TT_ANG(K,3,1)*VINF(3) )*VEE
      ASYS(IEQ,JETH)= -(  TT_ANG(K,1,2)*VINF(1)
     &                  + TT_ANG(K,2,2)*VINF(2)
     &                  + TT_ANG(K,3,2)*VINF(3) )*VEE
      ASYS(IEQ,JEPS)= -(  TT_ANG(K,1,3)*VINF(1)
     &                  + TT_ANG(K,2,3)*VINF(2)
     &                  + TT_ANG(K,3,3)*VINF(3) )*VEE
C
C---- y-velocity
      IEQ = JEY
      K = 2
      RSYS(IEQ)     = -(  TT(K,1)*VINF(1)
     &                  + TT(K,2)*VINF(2)
     &                  + TT(K,3)*VINF(3) )*VEE
      ASYS(IEQ,JEU) =     TT(K,1)
      ASYS(IEQ,JEV) =     TT(K,2)
      ASYS(IEQ,JEW) =     TT(K,3)
      ASYS(IEQ,JEPH)= -(  TT_ANG(K,1,1)*VINF(1)
     &                  + TT_ANG(K,2,1)*VINF(2)
     &                  + TT_ANG(K,3,1)*VINF(3) )*VEE
      ASYS(IEQ,JETH)= -(  TT_ANG(K,1,2)*VINF(1)
     &                  + TT_ANG(K,2,2)*VINF(2)
     &                  + TT_ANG(K,3,2)*VINF(3) )*VEE
      ASYS(IEQ,JEPS)= -(  TT_ANG(K,1,3)*VINF(1)
     &                  + TT_ANG(K,2,3)*VINF(2)
     &                  + TT_ANG(K,3,3)*VINF(3) )*VEE
C
C---- z-velocity
      IEQ = JEZ
      K = 3
      RSYS(IEQ)     = -(  TT(K,1)*VINF(1)
     &                  + TT(K,2)*VINF(2)
     &                  + TT(K,3)*VINF(3) )*VEE
      ASYS(IEQ,JEU) =     TT(K,1)
      ASYS(IEQ,JEV) =     TT(K,2)
      ASYS(IEQ,JEW) =     TT(K,3)
      ASYS(IEQ,JEPH)= -(  TT_ANG(K,1,1)*VINF(1)
     &                  + TT_ANG(K,2,1)*VINF(2)
     &                  + TT_ANG(K,3,1)*VINF(3) )*VEE
      ASYS(IEQ,JETH)= -(  TT_ANG(K,1,2)*VINF(1)
     &                  + TT_ANG(K,2,2)*VINF(2)
     &                  + TT_ANG(K,3,2)*VINF(3) )*VEE
      ASYS(IEQ,JEPS)= -(  TT_ANG(K,1,3)*VINF(1)
     &                  + TT_ANG(K,2,3)*VINF(2)
     &                  + TT_ANG(K,3,3)*VINF(3) )*VEE
C
      RETURN
      END ! APPMAT



      SUBROUTINE SYSSHO(LU,ASYS,BSYS,RSYS,NSYS)
C------------------------------------------------------------------
C     Prints out state-system matrices "A" and "B" 
C     In an organized manner.
C------------------------------------------------------------------
      INCLUDE 'AVL.INC'
      REAL*8 ASYS(JEMAX,JEMAX),BSYS(JEMAX,NDMAX),RSYS(JEMAX)
      REAL*8 USGN(JEMAX)
C
      DO I = 1, NSYS
        USGN(I) = 1.0
      ENDDO
      USGN(JEU) = -1.0
      USGN(JEW) = -1.0
      USGN(JEP) = -1.0
      USGN(JER) = -1.0
      USGN(JEX) = -1.0
      USGN(JEZ) = -1.0
C
      WRITE(LU,*)
      WRITE(LU,1100)
     &'     u         w         q        the   ',
     &'     v         p         r        phi   ',
     &'     x         y         z        psi   ',
     & (DNAME(N), N=1, NCONTROL)
C      1234567890123456789012345678901234567890
 1100 FORMAT(1X,A,A,A,1X,'|',2X,12A12)
C
      DO I = 1, NSYS
        WRITE(LU,1200) 
     &     (ASYS(I,J)*USGN(I)*USGN(J), J=1, NSYS),
     &     (BSYS(I,N)*USGN(I)        , N=1, NCONTROL)
c     &   ,   RSYS(I)*USGN(I)
 1200   FORMAT(1X,12F10.4,3X,12G12.4)
      ENDDO
C
      RETURN
      END ! SYSSHO


      SUBROUTINE EIGSOL(INFO,IR,ETOL,ASYS,NSYS)
C------------------------------------------------------------------
C     Computes eigenvalues and eigenvectors for run case IR.
C     Current forces and derivatives are assumed to be correct.
C------------------------------------------------------------------
      INCLUDE 'AVL.INC'
      REAL*8 ASYS(JEMAX,JEMAX)
C
      REAL*8 WR(JEMAX),WI(JEMAX),WVEC(JEMAX,JEMAX)
      REAL*8 WORK(JEMAX)
      INTEGER IWORK(JEMAX)
      LOGICAL LTERR
C
C---- call EISPACK's Real/General eigenvalue routine
C     ICALC = 0   ! get eigenvalues only
      ICALC = 1   ! get eigenvalues and eigenvectors
      CALL RG(JEMAX,NSYS,ASYS,WR,WI,ICALC,WVEC,IWORK,WORK,IERR)
C
C-------------------------------------------------------
C---- store goodies
C
      VEE = PARVAL(IPVEE,IR)
      BREFD = BREF*UNITL
      ETOLSQ = (ETOL*VEE/BREFD)**2
C
      KEIG = 0
C
      DO 100 J=1, NSYS
C------ don't store eigenvalue smaller than tolerance
        EMAGSQ = WR(J)**2 + WI(J)**2
        IF(EMAGSQ .LT. ETOLSQ) GO TO 100
C
        KEIG = KEIG + 1
        EVAL(KEIG,IR) = CMPLX( WR(J), WI(J) )
C
        IF    (WI(J) .EQ. 0.0) THEN
C------- real eigenvalue... just store real eigenvector
         DO I = 1, NSYS
           EVR = WVEC(I,J)
           EVI = 0.
           EVEC(I,KEIG,IR) = CMPLX( EVR , EVI )
         ENDDO
C
        ELSEIF(WI(J) .GT. 0.0) THEN
C------- positive imaginary part of eigenvalue... store complex eigenvector
         JP1 = MIN( J+1 , NSYS )
         DO I = 1, NSYS
           EVR = WVEC(I,J)
           EVI = WVEC(I,JP1)
           EVEC(I,KEIG,IR) = CMPLX( EVR , EVI )
         ENDDO
C
        ELSEIF(WI(J) .LT. 0.0) THEN
C------- negative imaginary part of eigenvalue... store complex eigenvector
         JM = MAX( J-1 , 1 )
         DO I = 1, NSYS
           EVR =  WVEC(I,JM)
           EVI = -WVEC(I,J)
           EVEC(I,KEIG,IR) = CMPLX( EVR , EVI )
         ENDDO
        ENDIF
 100  CONTINUE
C
      NEIGEN(IR) = KEIG
C
      RETURN
      END ! EIGSOL



      SUBROUTINE RUNLST(LU,IRE)
      INCLUDE 'AVL.INC'
      CHARACTER*1 RSYM
C
      WRITE(LU,1050)
 1050 FORMAT(1X,' ')
      WRITE(LU,1100) '  run  ',
     &    PARNAM(IPALFA),
     &    PARNAM(IPBETA), 
     &    PARNAM(IPCL  ), 
     &    PARNAM(IPCD0 ), 
     &    PARNAM(IPPHI ), 
     &    PARNAM(IPVEE ), 
     &    PARNAM(IPRHO ), 
     &    PARNAM(IPRAD ),
     &    PARNAM(IPFAC ),
     &    PARNAM(IPXCG ),
     &    PARNAM(IPZCG ),
     &    PARNAM(IPMASS),
     &    PARNAM(IPIXX ),
     &    PARNAM(IPIYY ),
     &    PARNAM(IPIZZ ) 
C
      WRITE(LU,1100) '       ',
     &    PARUNCH(IPALFA),
     &    PARUNCH(IPBETA), 
     &    PARUNCH(IPCL  ), 
     &    PARUNCH(IPCD0 ), 
     &    PARUNCH(IPPHI ), 
     &    PARUNCH(IPVEE ), 
     &    PARUNCH(IPRHO ), 
     &    PARUNCH(IPRAD ),
     &    PARUNCH(IPFAC ),
     &    PARUNCH(IPXCG ),
     &    PARUNCH(IPZCG ),
     &    PARUNCH(IPMASS),
     &    PARUNCH(IPIXX ),
     &    PARUNCH(IPIYY ),
     &    PARUNCH(IPIZZ ) 
C
 1100 FORMAT(1X,A, 20(1X,A9))
C
      DO IR = 1, NRUN
        IF(IRE.EQ.IR .OR. IRE.EQ.0) THEN
         RSYM = '>'
        ELSE
         RSYM = ' '
        ENDIF
        WRITE(LU,1300) RSYM, IR, 
     &    PARVAL(IPALFA,IR),
     &    PARVAL(IPBETA,IR), 
     &    PARVAL(IPCL  ,IR), 
     &    PARVAL(IPCD0 ,IR), 
     &    PARVAL(IPPHI ,IR), 
     &    PARVAL(IPVEE ,IR), 
     &    PARVAL(IPRHO ,IR), 
     &    PARVAL(IPRAD ,IR),
     &    PARVAL(IPFAC ,IR),
     &    PARVAL(IPXCG ,IR),
     &    PARVAL(IPZCG ,IR),
     &    PARVAL(IPMASS,IR),
     &    PARVAL(IPIXX ,IR),
     &    PARVAL(IPIYY ,IR),
     &    PARVAL(IPIZZ ,IR) 
 1300   FORMAT(1X,A, I3, 2X, 20G10.3)
      ENDDO
C
      RETURN
      END ! RUNLST



      SUBROUTINE PARMOD(IRE)
      INCLUDE 'AVL.INC'
C
      CHARACTER*2 CNUM
C
      CHARACTER*4 COM
      CHARACTER*80 CARG, PROMPT, RTNEW
      LOGICAL ERROR, REPKEY
C
      INTEGER IINP(10)
      REAL RINP(10)
C
      IF(IRE.EQ.0) THEN
       IR = 1
      ELSE
       IR = IRE
      ENDIF
C
C
 1000 FORMAT(A)
C
 2000 FORMAT(/'     Parameters of run case ',A,':  ', A)
 2005 FORMAT( '     ', A
     &       /'     =================================================')
 2105 FORMAT(6X, A, G10.4, 2X, A)
 2110 FORMAT(6X, A, A )
C
C--------------------------------------------------------------------------
C----- jump back here just for menu
 10    CONTINUE
       CALL CFRAC(IR,NRUN,PROMPT,NPR)
       WRITE(*,2000) PROMPT(1:NPR),RTITLE(IR)
C
       PHI   = PARVAL(IPPHI ,IR)
       THE   = PARVAL(IPTHE ,IR)
       MACH  = PARVAL(IPMACH,IR)
       VEE   = PARVAL(IPVEE ,IR)
       RHO   = PARVAL(IPRHO ,IR)
       GEE   = PARVAL(IPGEE ,IR)
       RMASS = PARVAL(IPMASS,IR)
       RIXX  = PARVAL(IPIXX ,IR)
       RIYY  = PARVAL(IPIYY ,IR)
       RIZZ  = PARVAL(IPIZZ ,IR)
       XCG   = PARVAL(IPXCG ,IR)
       YCG   = PARVAL(IPYCG ,IR)
       ZCG   = PARVAL(IPZCG ,IR)
       CD0   = PARVAL(IPCD0 ,IR)
       DCL_A = PARVAL(IPCLA ,IR)
       DCL_U = PARVAL(IPCLU ,IR)
       DCM_A = PARVAL(IPCMA ,IR)
       DCM_U = PARVAL(IPCMU ,IR)
C
       WRITE(*,2105) 'B  bank      = ', PHI  , 'deg'
       WRITE(*,2105) 'E  elevation = ', THE  , 'deg'
       WRITE(*,2105) 'MN Mach no.  = ', MACH , ' '
       WRITE(*,2105) 'V  velocity  = ', VEE  , UNCHV(1:NUV)
       WRITE(*,2105) 'D  air dens. = ', RHO  , UNCHD(1:NUD)
       WRITE(*,2105) 'G  grav.acc. = ', GEE  , UNCHA(1:NUA)
       WRITE(*,2105) 'M  mass      = ', RMASS, UNCHM(1:NUM)
       WRITE(*,2105) 'IX Ixx       = ', RIXX , UNCHI(1:NUI)
       WRITE(*,2105) 'IY Iyy       = ', RIYY , UNCHI(1:NUI)
       WRITE(*,2105) 'IZ Izz       = ', RIZZ , UNCHI(1:NUI)
       WRITE(*,2105) 'X  X_cg      = ', XCG  , 'Lunit'
       WRITE(*,2105) 'Y  Y_cg      = ', YCG  , 'Lunit'
       WRITE(*,2105) 'Z  Z_cg      = ', ZCG  , 'Lunit'
       WRITE(*,2105) 'CD CDo       = ', CD0  , ' '
       WRITE(*,2105) 'LA dCL_a     = ', DCL_A, ' '
       WRITE(*,2105) 'LU dCL_u     = ', DCL_U, ' '
       WRITE(*,2105) 'MA dCM_a     = ', DCM_A, ' '
       WRITE(*,2105) 'MU dCM_u     = ', DCM_U, ' '
C
       CALL ASKC('     Enter parameter, value  (or  # - + N )',COM,CARG)
C
       IF(COM.EQ.'    ') THEN
C------ just a Return entered... go back
        RETURN
       ENDIF
C
C------------------------------------------------------
C---- check for run case commands
      IF(COM.EQ.'+   ') THEN
C----- add new case after current one
C
       IF(NRUN.EQ.NRMAX) THEN
        WRITE(*,*)
        WRITE(*,*) '* Run case array limit NRMAX reached'
       ELSE
        NRUN = NRUN + 1
C
        DO JR = NRUN, IR+1, -1
          CALL RCOPY(JR,JR-1)
        ENDDO
        WRITE(*,*) 'Initializing new run case from current one'
C        
        IR = IR + 1
       ENDIF
C
       GO TO 10
C
      ELSEIF(COM.EQ.'-   ') THEN
C----- delete current case
C
       IF(NRUN.LE.1) THEN
        WRITE(*,*)
        WRITE(*,*) '* Cannot delete one remaining run case'
       ELSE
        DO JR = IR, NRUN-1
          CALL RCOPY(JR,JR+1)
        ENDDO
        NRUN  = NRUN - 1
        IRUN  = MAX( 1 , MIN( IRUN  , NRUN ) )
        IRUNE = MAX( 1 , MIN( IRUNE , NRUN ) )
       ENDIF
C
       GO TO 10
C
      ENDIF
C
C------------------------------------------------------
C---- see if command is an integer (new run case selection)
      NINP = 1
      CALL GETINT(COM,IINP,NINP,ERROR)
      IF(.NOT.ERROR .AND. NINP.GE.1 
     &  .AND. COM(1:1).NE.'T'
     &  .AND. COM(1:1).NE.'F' ) THEN
C----- command is an integer... new case index?
       IF(IINP(1).LT.1 .OR. IINP(1).GT.NRUN) THEN
        WRITE(*,*)
        WRITE(*,*) '* Selected new run case is not defined'
        GO TO 10
       ELSE
C------ valid new run case selected... go back to top of menu
        IR = IINP(1)
        GO TO 10
       ENDIF
      ENDIF
C
C
C------------------------------------------------------
C---- extract argument, if any
      NINP = 1
      CALL GETFLT(CARG,RINP,NINP,ERROR)
C
C
      IF    (COM(1:1) .EQ. COM(2:2)) THEN
       REPKEY = .TRUE.
       IR1 = 1
       IR2 = NRUN
       COM(2:2) = ' '
      ELSEIF(COM(1:2) .EQ. COM(3:4)) THEN
       REPKEY = .TRUE.
       IR1 = 1
       IR2 = NRUN
       COM(3:4) = '  '
      ELSE
       REPKEY = .FALSE.
       IR1 = IR
       IR2 = IR
      ENDIF
C
C==============================================================================
C---- now decode regular parameter value commands
C
C------------------------------------
      IF(COM(1:2) .EQ. 'B ') THEN
 11     CONTINUE
        IF(NINP.GE.1) THEN
         PHI = RINP(1)
        ELSE
         CALL ASKR('      Enter bank angle^',PHI)
        ENDIF
C
        IF(PHI.LE.-90.0.OR.PHI.GE.90.0) THEN
         WRITE(*,*) '    * Must have  -90 < bank < +90'
         NINP = 0
         GO TO 11
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPPHI,JR) = PHI
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'E ') THEN
 21     CONTINUE
        IF(NINP.GE.1) THEN
         THE = RINP(1)
        ELSE
         CALL ASKR('      Enter elevation angle^',THE)
        ENDIF
C
        IF(THE.LE.-90.0.OR.THE.GE.90.0) THEN
         WRITE(*,*) '    * Must have  -90 < elevation < +90'
         NINP = 0
         GO TO 21
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPTHE,JR) = THE
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'MN') THEN
 24     CONTINUE
        IF(NINP.GE.1) THEN
         MACH = RINP(1)
        ELSE
         CALL ASKR('      Enter Mach number^',MACH)
        ENDIF
C
        IF(MACH .LT. 0.0 .OR. MACH .GT. 0.999) THEN
         WRITE(*,*) '    * Must have 0 < Mach < 0.999'
         GO TO 24
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPMACH,JR) = MACH
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'V ') THEN
 31     CONTINUE
        IF(NINP.GE.1) THEN
         VEE = RINP(1)
        ELSE
         CALL ASKR('      Enter velocity^',VEE)
        ENDIF
C
        IF(VEE.LE.0.0) THEN
         WRITE(*,*) '    * Must have  V > 0'
         NINP = 0
         GO TO 31
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPVEE,JR) = VEE
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'M ') THEN
 41     CONTINUE
        IF(NINP.GE.1) THEN
         RMASS = RINP(1)
        ELSE
         CALL ASKR('      Enter mass^',RMASS)
        ENDIF
C
        IF(RMASS.LE.0.0) THEN
         WRITE(*,*) '    * Must have  m > 0'
         NINP = 0
         GO TO 41
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPMASS,JR) = RMASS
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'IX') THEN
 42     CONTINUE
        IF(NINP.GE.1) THEN
         RIXX = RINP(1)
        ELSE
         CALL ASKR('      Enter Ixx^',RIXX)
        ENDIF
C
        IF(RIXX.LE.0.0) THEN
         WRITE(*,*) '    * Must have  Ixx > 0'
         NINP = 0
         GO TO 42
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPIXX,JR) = RIXX
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'IY') THEN
 43     CONTINUE
        IF(NINP.GE.1) THEN
         RIYY = RINP(1)
        ELSE
         CALL ASKR('      Enter Iyy^',RIYY)
        ENDIF
C
        IF(RIYY.LE.0.0) THEN
         WRITE(*,*) '    * Must have  Iyy > 0'
         NINP = 0
         GO TO 43
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPIYY,JR) = RIYY
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'IZ') THEN
 44     CONTINUE
        IF(NINP.GE.1) THEN
         RIZZ = RINP(1)
        ELSE
         CALL ASKR('      Enter Izz^',RIZZ)
        ENDIF
C
        IF(RIZZ.LE.0.0) THEN
         WRITE(*,*) '    * Must have  Izz > 0'
         NINP = 0
         GO TO 44
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPIZZ,JR) = RIZZ
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'D ') THEN
 51     CONTINUE
        IF(NINP.GE.1) THEN
         RHO = RINP(1)
        ELSE
         CALL ASKR('      Enter air density^',RHO)
        ENDIF
C
        IF(RHO.LE.0.0) THEN
         WRITE(*,*) '    * Must have  rho > 0'
         NINP = 0
         GO TO 51
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPRHO,JR) = RHO
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'G ') THEN
 61     CONTINUE
        IF(NINP.GE.1) THEN
         GEE = RINP(1)
        ELSE
         CALL ASKR('      Enter gravity^',GEE)
        ENDIF
C
        IF(GEE.LE.0.0) THEN
         WRITE(*,*) '    * Must have  g > 0'
         NINP = 0
         GO TO 61
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPGEE,JR) = GEE
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'X ') THEN
        IF(NINP.GE.1) THEN
         XCG = RINP(1)
        ELSE
         CALL ASKR('      Enter X_cg location^',XCG)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPXCG,JR) = XCG
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'Y ') THEN
        IF(NINP.GE.1) THEN
         YCG = RINP(1)
        ELSE
         CALL ASKR('      Enter Y_cg location^',YCG)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPYCG,JR) = YCG
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'Z ') THEN
        IF(NINP.GE.1) THEN
         ZCG = RINP(1)
        ELSE
         CALL ASKR('      Enter Z_cg location^',ZCG)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPZCG,JR) = ZCG
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'CD') THEN
        IF(NINP.GE.1) THEN
         CD0 = RINP(1)
        ELSE
         CALL ASKR('      Enter profile CDo^',CD0)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPCD0,JR) = CD0
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'LA') THEN
        IF(NINP.GE.1) THEN
         DCL_A = RINP(1)
        ELSE
         CALL ASKR('      Enter added CL_a^',DCL_A)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPCLA,JR) = DCL_A
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'LU') THEN
        IF(NINP.GE.1) THEN
         DCL_U = RINP(1)
        ELSE
         CALL ASKR('      Enter added CL_u^',DCL_U)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPCLU,JR) = DCL_U
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'MA') THEN
        IF(NINP.GE.1) THEN
         DCM_A = RINP(1)
        ELSE
         CALL ASKR('      Enter added CM_a^',DCM_A)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPCMA,JR) = DCM_A
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:2) .EQ. 'MU') THEN
        IF(NINP.GE.1) THEN
         DCM_U = RINP(1)
        ELSE
         CALL ASKR('      Enter added CM_u^',DCM_U)
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPCMU,JR) = DCM_U
        ENDDO
C
C------------------------------------------------------
      ELSEIF(COM(1:1) .EQ. 'N') THEN
C------ change name of run case
        IF(CARG.NE.' ') THEN
         RTITLE(IR) = CARG
        ELSE
         WRITE(*,830) RTITLE(IR)
 830     FORMAT(/' Enter run case name:  ', A)
         READ(*,1000) RTNEW
         IF(RTNEW.NE.' ') RTITLE(IR) = RTNEW
        ENDIF
C
      ELSE
        WRITE(*,*) '     * Unrecognized parameter'
C
      ENDIF
C
      IF(REPKEY) WRITE(*,*) '      Value set for all run cases'
      GO TO 10
C
      END ! PARMOD
