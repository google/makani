C***********************************************************************
C    Module:  aoper.f
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

      SUBROUTINE OPER
C---------------------------------------
C     Main driver routine for AVL
C---------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      LOGICAL ERROR, LCERR, LCERRI, LWRIT, LMATCH
C
      CHARACTER*1 ANS
      CHARACTER*2 OPT
      CHARACTER*4 COMAND, ITEMC
      CHARACTER*80 FNOUT, FNDER, FNNEW
      CHARACTER*80 LINE, FNVB, COMARG, CRUN, PROMPT, RTNEW
      CHARACTER*50 SATYPE, ROTTYPE
C
      LOGICAL LOWRIT
C
      REAL    RINPUT(20), RINP(20)
      INTEGER IINPUT(20), IINP(20)
C
      IF(.NOT.LGEO) THEN
       WRITE(*,*)
       WRITE(*,*) '* Configuration not defined'
       RETURN
      ENDIF
C
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
      LPLOT = .FALSE.
      LWRIT = .FALSE.
      LSOL  = .FALSE.
C
      FNOUT = ' '
C
C=================================================================
C---- start of user interaction loop
 800  CONTINUE
C
      LCERR = .FALSE.
C
C
      CALL CFRAC(IRUN,NRUN,CRUN,NPR)
C
      WRITE(*,1050) CRUN(1:NPR), RTITLE(IRUN)
      CALL CONLST(IRUN)
      WRITE(*,1052)
C
 1050 FORMAT(
     &  /' Operation of run case ',A,':  ', A
     &  /' ==========================================================')
C
 1052 FORMAT(
     &  /'  C1  set level or banked  horizontal flight constraints'
     &  /'  C2  set steady pitch rate (looping) flight constraints'
     &  /'  M odify parameters                                    '
     & //' "#" select  run case          L ist defined run cases   '
     &  /'  +  add new run case          S ave run cases to file   '
     &  /'  -  delete  run case          F etch run cases from file'
     &  /'  N ame current run case       W rite forces to file     '
     & //' eX ecute run case             I nitialize variables     '
     & //'  G eometry plot               T refftz Plane plot       '
     & //'  ST  stability derivatives    FT  total   forces        '
     &  /'  SB  body-axis derivatives    FN  surface forces        '
     &  /'  RE  reference quantities     FS  strip   forces        '
     &  /'  DE  design changes           FE  element forces        '
     &  /'  O ptions                     FB  body forces           '
     &  /'                               HM  hinge moments         '
     &  /'                               VM  strip shear,moment    ')
C 
C   A B C D E F G H I J K L M N O P Q R S T U V W X Y Z
C   x x x x   x x   x     x x x x x   x x x   x x x x

 810  CONTINUE
      PROMPT = ' .OPER (case ' // CRUN(1:NPR) // ')^'
      CALL ASKC(PROMPT,COMAND,COMARG)
C
C------------------------------------------------------
      IF    (COMAND.EQ.'    ') THEN
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL CLRZOOM
       RETURN
C
      ELSEIF(COMAND.EQ.'?   ') THEN
       GO TO 800
C
      ENDIF
C
C------------------------------------------------------
C---- check for run case commands
      IF(COMAND.EQ.'+   ') THEN
C----- add new case after current one
C
       IF(NRUN.EQ.NRMAX) THEN
        WRITE(*,*)
        WRITE(*,*) '* Run case array limit NRMAX reached'
       ELSE
        NRUN = NRUN + 1
C
        DO JR = NRUN, IRUN+1, -1
          CALL RCOPY(JR,JR-1)
        ENDDO
        WRITE(*,*) 'Initializing new run case from current one'
C        
        IRUN = IRUN + 1
       ENDIF
C
       GO TO 800
C
      ELSEIF(COMAND.EQ.'-   ') THEN
C----- delete current case
C
       IF(NRUN.LE.1) THEN
        WRITE(*,*)
        WRITE(*,*) '* Cannot delete one remaining run case'
       ELSE
        DO JR = IRUN, NRUN-1
          CALL RCOPY(JR,JR+1)
        ENDDO
        NRUN  = NRUN - 1
        IRUN  = MAX( 1 , MIN( IRUN  , NRUN ) )
        IRUNE = MAX( 1 , MIN( IRUNE , NRUN ) )
       ENDIF
C
       GO TO 800
C
      ENDIF
C
C------------------------------------------------------
C---- first check if point index was input
      IF(INDEX('0123456789',COMAND(1:1)) .NE. 0) THEN
        READ(COMAND,*,ERR=8) IRUN
        IF(IRUN.LT.1 .OR. IRUN.GT.NRUN) THEN
         IRUN = MAX( 1 , MIN(NRUN,IRUN) )
         WRITE(*,*) '* Run case index was limited to available range'
        ENDIF
        GO TO 800
C
 8      WRITE(*,*) '* Invalid command: ', COMAND
        GO TO 800
      ENDIF
C
C------------------------------------------------------
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
 14   CONTINUE
C------------------------------------------------------
C---- check for parameter toggle/set command
      CALL CONSET(COMAND,COMARG,LMATCH,IRUN)
C
      IF(LMATCH) THEN
C----- match found... go back to OPER menu
       GO TO 800
      ENDIF
C
C------------------------------------------------------
C---- check for trim set command
      CALL TRMSET(COMAND,COMARG,LMATCH,IRUN)
C
      IF(LMATCH) THEN
C----- match found... go back to OPER menu
       GO TO 800
      ENDIF
C
C------------------------------------------------------
C---- pick up here to try decoding for remaining commands
C
      IF(COMAND .EQ. 'X   ') THEN
C------ execute calculation
        IF(LCERR) THEN
         WRITE(*,*) '** Flow solution is not possible.'
         WRITE(*,*) '** Cannot impose a constraint more than once.'
         GO TO 800
        ENDIF
C
        IRUN0 = IRUN
C
        XYZREF(1) = PARVAL(IPXCG,IRUN)
        XYZREF(2) = PARVAL(IPYCG,IRUN)
        XYZREF(3) = PARVAL(IPZCG,IRUN)
        CDREF     = PARVAL(IPCD0,IRUN)
C
        INFO = 1
        CALL EXEC(NITMAX,INFO,IRUN)
        IF(.NOT.LSOL) GO TO 810
C
        IF(LPTOT)   CALL OUTTOT(6)
        IF(LPSURF)  CALL OUTSURF(6)
        IF(LPSTRP)  CALL OUTSTRP(6)
        IF(LPELE)   CALL OUTELE(6)
        IF(LPHINGE) CALL OUTHINGE(6)
C
C------------------------------------------------------
      ELSEIF(COMAND .EQ. 'XX  ') THEN
C------ execute calculation for all run cases
        DO 24 IR = 1, NRUN
C-------- check for well-posedness
          LCERRI = .FALSE.
          DO IV = 1, NVTOT
            IC = ICON(IV,IR)
            DO JV = 1, NVTOT
              IF(IV.NE.JV .AND. ICON(IV,IR).EQ.ICON(JV,IR)) THEN
               LCERRI = .TRUE.
              ENDIF
            ENDDO
          ENDDO
          IF(LCERRI) THEN
           WRITE(*,*) '** Run case', IR,' ...'
           WRITE(*,*) '** Flow solution is not possible.'
           WRITE(*,*) '** Cannot impose a constraint more than once.'
           GO TO 24          
          ENDIF
C
          INFO = 1
          CALL EXEC(NITMAX,INFO,IR)
          IF(.NOT.LSOL) GO TO 24
C
          IF(LPTOT)  CALL OUTTOT(6)
          IF(LPSURF) CALL OUTSURF(6)
          IF(LPSTRP) CALL OUTSTRP(6)
          IF(LPELE)  CALL OUTELE(6)
          IF(LPHINGE) CALL OUTHINGE(6)
 24     CONTINUE
C
C------------------------------------------------------
      ELSEIF(COMAND .EQ. 'M   ') THEN
        CALL PARMOD(IRUN)
C
C------------------------------------------------------
      ELSEIF(COMAND .EQ. 'N   ') THEN
C------ change name of run case
        IF(COMARG.NE.' ') THEN
         RTITLE(IRUN) = COMARG
        ELSE
         WRITE(*,830) RTITLE(IRUN)
 830     FORMAT(/' Enter run case name:  ', A)
         READ(*,1000) RTNEW
         IF(RTNEW.NE.' ') RTITLE(IRUN) = RTNEW
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'DE  ') THEN
C------ design changes
        IF(NDESIGN.EQ.0) THEN
         WRITE(*,*) '* No design parameters are declared'
         GO TO 810
        ENDIF
C
 30     CONTINUE
        WRITE(*,1036)
        DO K = 1, NDESIGN
          WRITE(*,1037) K, GNAME(K), DELDES(K)
        ENDDO
 1036   FORMAT(/' ================================================'
     &         /' Current design parameter changes:' /
     &         /'    k   Parameter      change')
CCC              x1234xxx123456789012345612345678901234
 1037   FORMAT(1X, I4,3X, A, G14.5) 
C
        WRITE(*,*)
 35     WRITE(*,*) 'Enter  k, design changes (<return> if done) ...'
 37     READ (*,1000) LINE
        CALL STRIP(LINE,NLIN)
        IF(LINE(1:1).EQ.'?') GO TO 30
        IF(LINE(1:1).EQ.' ') GO TO 800
C
        NINP = 40
        CALL GETFLT(LINE,RINP,NINP,ERROR)
        IF(ERROR) THEN
         WRITE(*,*) '* Bad input'
         GO TO 35
        ENDIF
C
        DO I = 1, NINP, 2
          N = INT(RINP(I))
          IF(N.LT.1 .OR. N.GT.NDESIGN) THEN
           WRITE(*,*) 'Index k out of bounds. Input ignored.'
          ENDIF
          DELDES(N) = RINP(I+1)
          LSOL = .FALSE.
          GO TO 37
        ENDDO
        GO TO 30
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'I   ') THEN
C------ clear operating parameters
        ALFA = 0.
        BETA = 0.
        WROT(1) = 0.
        WROT(2) = 0.
        WROT(3) = 0.
C
        DO N = 1, NCONTROL
          DELCON(N) = 0.
        ENDDO
C
        DO N = 1, NDESIGN
          DELDES(N) = 0.
        ENDDO
C
        LSOL = .FALSE.
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'G   ') THEN
C------ plot geometry
        CALL PLOTVL(AZIMOB, ELEVOB, TILTOB, ROBINV)
        LPLOT = .TRUE.
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'T   ') THEN
C------ plot spanloadings in Trefftz plane
        CALL PLOTTP
        LPLOT = .TRUE.
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'FT  ') THEN
C------ print total forces
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTTOT(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'FN  ') THEN
C------ print surface forces
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTSURF(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'FS  ') THEN
C------ print strip forces
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTSTRP(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'FE  ') THEN
C------ print vortex element forces
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTELE(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'FB  ') THEN
C------ print body forces
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTBODY(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'HM  ') THEN
C------ print hinge moments
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTHINGE(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'VM  ') THEN
C------ calculate and print shear and bending on surfaces
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTVM(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'CN  ') THEN
C------ print a spanloading file
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL OUTCNC(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'L   ') THEN
C------ list run cases
        LU = 6
        CALL RUNSAV(LU)
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'S   ') THEN
C------ save run case file
        CALL STRIP(FRNDEF,NFR)
        WRITE(*,2040) FRNDEF(1:NFR)
 2040   FORMAT(' Enter run case filename: ', A)
        READ (*,1000) FNNEW
C
        IF(FNNEW.NE.' ') FRNDEF = FNNEW
        OPEN(LURUN,FILE=FRNDEF,STATUS='OLD',ERR=42)
C
        IF(LOWRIT(FRNDEF)) THEN
         REWIND(LURUN)
         GO TO 45
        ELSE
         WRITE(*,*) 'Run cases not saved'
         GO TO 810
        ENDIF
C
 42     OPEN(LURUN,FILE=FRNDEF,STATUS='NEW')
C
 45     CALL RUNSAV(LURUN)
        CLOSE(LURUN)
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'F   ') THEN
C------ fetch run case file
        CALL STRIP(FRNDEF,NFR)
        WRITE(*,2050) FRNDEF(1:NFR)
 2050   FORMAT(' Enter run case filename: ', A)
        READ (*,1000) FNNEW
        IF(FNNEW.NE.' ') FRNDEF = FNNEW
C
        CALL RUNGET(LURUN,FRNDEF,ERROR)
        IF(ERROR) THEN
         GO TO 810
        ELSE
         WRITE(*,2055) (IR, RTITLE(IR), IR=1, NRUN)
 2055    FORMAT(' Run cases read in ...',
     &          100(/1X,I4,': ',A))
        ENDIF
C
        IRUN = MIN( IRUN, NRUN )
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'ST  ') THEN
C------ create stability derivatives
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL DERMATS(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'SM  ') THEN
C------ create stability derivatives
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL DERMATM(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'SB  ') THEN
C------ create stability derivatives
        IF(LSOL) THEN
         CALL GETFILE(LU,COMARG)
C
         IF(LU.LE.-1) THEN
          WRITE(*,*) '* Filename error *'
         ELSEIF(LU.EQ.0) THEN
          WRITE(*,*) '* Data not written'
         ELSE
          CALL DERMATB(LU)
          IF(LU.NE.5 .AND. LU.NE.6) CLOSE(LU)
         ENDIF
C
        ELSE
         WRITE(*,*) '* Execute flow calculation first!'
        ENDIF
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'W   ') THEN
C------ write force  data to a file
        CALL STRIP(FNOUT,NFN)
        WRITE(*,1080) FNOUT(1:NFN)
 1080   FORMAT('Enter forces output file: ', A)
        READ (*,1000) FNNEW
C
        IF(FNNEW.NE.' ') THEN
C-------- new filename was entered...
C-------- if previous file is open, close it
          IF(LWRIT) CLOSE(LUOUT)
          FNOUT = FNNEW
C-------- open new file and write header
          OPEN(LUOUT,FILE=FNOUT,STATUS='UNKNOWN')
          LWRIT = .TRUE.
C
        ELSE
C-------- just a <return> was entered...
          IF(.NOT.LWRIT) THEN
            WRITE(*,*) 'No action taken.'
            GO TO 800
          ENDIF
C
        ENDIF
C
        IF(LPTOT)   CALL OUTTOT(LUOUT)
        IF(LPSURF)  CALL OUTSURF(LUOUT)
        IF(LPSTRP)  CALL OUTSTRP(LUOUT)
        IF(LPELE)   CALL OUTELE(LUOUT)
ccc     IF(LPDERIV) CALL DERMAT(LUOUT)
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'RE  ') THEN
C------ Change reference data 
 89     WRITE(*,2090) SREF,CREF,BREF
 2090   FORMAT(/' ==========================='
     &         /'  S ref: ', G11.5,
     &         /'  C ref: ', G11.5,
     &         /'  B ref: ', G11.5 )
C
 90     CALL ASKC(' Select item,value^',ITEMC,COMARG)
 2100   FORMAT(' Enter new ',A,': ', $)
C
        IF(ITEMC.EQ.'    ') THEN
          GO TO 800
        ENDIF
C
        NINP = 1
        CALL GETFLT(COMARG,RINP,NINP,ERROR)
C
        IF    (INDEX('Ss',ITEMC(1:1)).NE.0) THEN
          IF(NINP.EQ.0) THEN
 91        WRITE(*,2100) 'reference area Sref'
           READ (*,*,ERR=91) SREF
          ELSE
           SREF = RINP(1)
          ENDIF
          LSOL = .FALSE.
C
        ELSEIF(INDEX('Cc',ITEMC(1:1)).NE.0) THEN
          IF(NINP.EQ.0) THEN
 92        WRITE(*,2100) 'reference chord Cref'
           READ (*,*,ERR=92) CREF
          ELSE
           CREF = RINP(1)
          ENDIF
          LSOL = .FALSE.
C
        ELSEIF(INDEX('Bb',ITEMC(1:1)).NE.0) THEN
          IF(NINP.EQ.0) THEN
 93        WRITE(*,2100) 'reference span Bref'
           READ (*,*,ERR=93) BREF
          ELSE
           BREF = RINP(1)
          ENDIF
          LSOL = .FALSE.
C
        ELSE
          WRITE(*,*) 'Item not recognized'
          GO TO 89
C
        ENDIF
        GO TO 89
C
C------------------------------------------------------
      ELSE IF(COMAND.EQ.'O   ') THEN
        CALL OPTGET


C------------------------------------------------------
      ELSE
        WRITE(*,*)
        WRITE(*,*) '* Option not recognized'
C
      ENDIF
      GO TO 800
C
      END ! OPER




      SUBROUTINE CONLST(IR)
      INCLUDE 'AVL.INC'
C
      CHARACTER*4 CHSS
C
      WRITE(*,1010)
C
      DO IV = 1, NVTOT
        IC = ICON(IV,IR)
        CHSS = '  '
        DO JV = 1, NVTOT
          IF(IV.NE.JV .AND. ICON(IV,IR).EQ.ICON(JV,IR)) THEN
           CHSS = '**'
          ENDIF
        ENDDO
        WRITE(*,1020) VARKEY(IV), CONNAM(IC), CONVAL(IC,IR), CHSS
      ENDDO
C
      WRITE(*,1030)
      RETURN
C      
 1010 FORMAT(
     &  /'  variable          constraint              '
     &  /'  ------------      ------------------------')
 10200 FORMAT(
     &   '  ',A,'  ->  ', A, '=', G12.4, 1X, A)
 1030 FORMAT(
     &   '  ------------      ------------------------')
      END ! CONLST



      SUBROUTINE CONSET(COMAND,COMARG,LMATCH,IR)
      INCLUDE 'AVL.INC'
      CHARACTER*(*) COMAND, COMARG
      LOGICAL LMATCH
C
      CHARACTER*80 PROMPT
      CHARACTER*4 ARROW
      REAL    RINP(20)
      INTEGER IINP(20)
      LOGICAL ERROR
C
C---- for control variable, first number in arg string should be part of command
      IF(COMAND(1:2) .EQ. 'D ') THEN
       COMAND(2:3) = COMARG(1:2)
       COMARG(1:2) = '  '
       CALL STRIP(COMARG,NARG)
      ENDIF
C
C---- length of non-blank part of command, if any
      KCLEN = INDEX(COMAND,' ') - 1
      IF(KCLEN.LE.0) KCLEN = LEN(COMAND)
C
C---- test command against variable keys, using only non-blank part of command
      DO IV = 1, NVTOT
        KVLEN = INDEX(VARKEY(IV),' ') - 1
        IF(KCLEN .EQ. KVLEN .AND.
     &     COMAND(1:KCLEN) .EQ. VARKEY(IV)(1:KVLEN)) GO TO 16
      ENDDO
C
C---- no variable key matched... go test for regular commands
      LMATCH = .FALSE.
      RETURN
C
C------------------------------------------------------
C---- found a variable-key match!
 16   CONTINUE
      LMATCH = .TRUE.
      CALL TOUPER(COMARG) 
C
C---- see if constraint was already specified as second command argument
      KCLEN = INDEX(COMARG,' ') - 1
      IF(KCLEN.LE.0) KCLEN = LEN(COMARG)
      KCLEN = MIN( KCLEN , LEN(CONKEY(1)) )
      DO IC = 1, NCTOT
        IF(COMARG(1:KCLEN) .EQ. CONKEY(IC)(1:KCLEN)) GO TO 18
      ENDDO
C
C---- constraint not given... get it from constraint-selection menu
      WRITE(*,1081)
      DO IC = 1, NCTOT
        IF(IC.EQ.ICON(IV,IR)) THEN
         ARROW = '->  '
        ELSE
         ARROW = '    '
        ENDIF
        WRITE(*,1082) ARROW, CONKEY(IC), CONNAM(IC), CONVAL(IC,IR)
      ENDDO
 1081 FORMAT(/'       constraint            value     '
     &       /'      - - - - - - - - - - - - - - - - -')
 1082 FORMAT( '   ', A, A, 2X, A, '=', G12.4)
C
      PROMPT= '      Select new  constraint,value  for '
     &        // VARNAM(IV) // '^'
      CALL ASKC(PROMPT,COMAND,COMARG)
      IF(COMAND.EQ.' ') RETURN
C
      IF(COMAND(1:2) .EQ. 'D ') THEN
       COMAND(2:3) = COMARG(1:2)
       COMARG(1:2) = '  '
       CALL STRIP(COMARG,NARG)
      ENDIF
C
C---- try to parse command again
      COMARG = COMAND(1:3) // ' ' // COMARG
      GO TO 16
C
C----------------------------------
C---- pick up here to set new constraint
 18   CONTINUE
C
C---- set new constraint index for selected variable IV
      ICON(IV,IR) = IC
C
C---- see if constraint value was already specified in command argument
      NINP = 1
      CALL GETFLT(COMARG(KCLEN+1:80),RINP,NINP,ERROR)
      IF(ERROR) NINP = 0
C
      IF(NINP.GE.1) THEN
C----- yep...  set constraint value to command argument
       CONVAL(IC,IR) = RINP(1)
      ELSE
C----- nope... get constraint value from user (current value is the default)
 19    WRITE(*,1090) CONNAM(IC), CONVAL(IC,IR)
 1090  FORMAT(/' Enter specified ', A,':', G12.4)
       CALL READR(1,CONVAL(IC,IR),ERROR)
       IF(ERROR) GO TO 19
      ENDIF
C
C---- go back to OPER menu
      RETURN
      END ! CONSET





      SUBROUTINE EXEC(NITER,INFO,IR)
C---------------------------------------------------
C     Solves for the flow condition specified by 
C     the global operating parameters:
C
C       CONVAL(ICALFA)     alpha (deg)
C       CONVAL(ICBETA)     beta  (deg)
C       CONVAL(ICROTX)     roll_rate * Bref / 2V
C       CONVAL(ICROTY)    pitch_rate * Cref / 2V
C       CONVAL(ICROTZ)      yaw_rate * Bref / 2V
C        .
C        .
C
C---------------------------------------------------
      INCLUDE 'AVL.INC'
      REAL VSYS(IVMAX,IVMAX), VRES(IVMAX), DDC(NDMAX), WORK(IVMAX)
      INTEGER IVSYS(IVMAX)
C
C---- convergence epsilon, max angle limit (radians)
      DATA EPS, DMAX / 0.00002, 1.0 /
C
      IF(LNASA_SA) THEN
C----- NASA Std. Stability axes, X fwd, Z down
       DIR = -1.0
      ELSE
C----- Geometric Stability axes, X aft, Z up
       DIR =  1.0
      ENDIF
C
      XYZREF(1) = PARVAL(IPXCG,IR)
      XYZREF(2) = PARVAL(IPYCG,IR)
      XYZREF(3) = PARVAL(IPZCG,IR)
C
      CDREF = PARVAL(IPCD0,IR)
C
      MACH = PARVAL(IPMACH,IR)
C
      IF(MACH.NE.AMACH) THEN
C----- new Mach number invalidates close to everything that's stored
       LAIC = .FALSE.
       LSRD = .FALSE.
       LSOL = .FALSE.
       LSEN = .FALSE.
      ENDIF
C
C---- set, factor AIC matrix and induced-velocity matrix (if they don't exist)
      CALL SETUP
C
      IF(NITER.GT.0) THEN
C----- might as well directly set operating variables if they are known
       IF(ICON(IVALFA,IR).EQ.ICALFA) ALFA    = CONVAL(ICALFA,IR)*DTR
       IF(ICON(IVBETA,IR).EQ.ICBETA) BETA    = CONVAL(ICBETA,IR)*DTR
       IF(ICON(IVROTX,IR).EQ.ICROTX) WROT(1) = CONVAL(ICROTX,IR)*2./BREF
       IF(ICON(IVROTY,IR).EQ.ICROTY) WROT(2) = CONVAL(ICROTY,IR)*2./CREF
       IF(ICON(IVROTZ,IR).EQ.ICROTZ) WROT(3) = CONVAL(ICROTZ,IR)*2./BREF
      ENDIF
C
C----- set GAM_U
ccc       WRITE(*,*) ' Solving for unit-freestream vortex circulations...'
      CALL GUCALC
C
C-------------------------------------------------------------
C---- calculate initial operating state
C
C---- set VINF() vector from initial ALFA,BETA
      CALL VINFAB
C
c      IF(NCONTROL.GT.0) THEN
cC----- set GAM_D
cccc       WRITE(*,*) ' Solving for vortex control-var sensitivities...'
c       CALL GDCALC(NCONTROL,LCONDEF,ENC_D,GAM_D)
c      ENDIF
C
c      IF(NDESIGN.GT.0) THEN
cC----- set GAM_G
cccc       WRITE(*,*) ' Solving for vortex  design-var sensitivities...'
c       CALL GDCALC(NDESIGN ,LDESDEF,ENC_G,GAM_G)
c      ENDIF
C
C---- sum AIC matrices to get GAM,SRC,DBL
      CALL GAMSUM
C
C---- sum AIC matrices to get WC,WV
      CALL VELSUM
C
C---- compute forces
      CALL AERO
C
C---- Newton loop for operating variables
      DO 190 ITER = 1, NITER
C
        IF(LSA_RATES) THEN
C-------- rates specified in NASA stability-axes, transform to body axes
          CA = COS(ALFA)
          SA = SIN(ALFA)
          CA_A = -SA
          SA_A =  CA
         ELSE
C-------- rates specified in body-axes, no transformation
          CA = 1.0
          SA = 0.0
          CA_A = 0.
          SA_A = 0.
        ENDIF
C
        DO K=1, IVMAX
          DO L=1, IVMAX
            VSYS(K,L) = 0.
          ENDDO
        ENDDO
C
C------ set up Newton system:  set constraints for all parameters
        DO 100 IV = 1, NVTOT
C
C-------- set index and value of constraint for this parameter
          IC = ICON(IV,IR)
C
C------------------------------------
          IF    (IC.EQ.ICALFA) THEN
           VRES(IV) = ALFA - CONVAL(IC,IR)*DTR
           VSYS(IV,IVALFA) = 1.0
C
C------------------------------------
          ELSEIF(IC.EQ.ICBETA) THEN
           VRES(IV) = BETA - CONVAL(IC,IR)*DTR
           VSYS(IV,IVBETA) = 1.0
C
C------------------------------------
          ELSEIF(IC.EQ.ICROTX) THEN
           VRES(IV) = (WROT(1)*CA + WROT(3)*SA)*DIR
     &              - CONVAL(IC,IR)*2.0/BREF
           VSYS(IV,IVROTX) = CA*DIR
           VSYS(IV,IVROTZ) = SA*DIR
           VSYS(IV,IVALFA) = (WROT(1)*CA_A + WROT(3)*SA_A)*DIR
C
C------------------------------------
          ELSEIF(IC.EQ.ICROTY) THEN
           VRES(IV) = WROT(2)
     &              - CONVAL(IC,IR)*2.0/CREF
           VSYS(IV,IVROTY) = 1.0
C
C------------------------------------
          ELSEIF(IC.EQ.ICROTZ) THEN
           VRES(IV) = (WROT(3)*CA - WROT(1)*SA)*DIR
     &              - CONVAL(IC,IR)*2.0/BREF
           VSYS(IV,IVROTX) = -SA*DIR
           VSYS(IV,IVROTZ) =  CA*DIR
           VSYS(IV,IVALFA) = (WROT(3)*CA_A - WROT(1)*SA_A)*DIR
C
C------------------------------------
          ELSEIF(IC.EQ.ICCL  ) THEN
           VRES(IV) = CLTOT - CONVAL(IC,IR)
           VSYS(IV,IVALFA) = CLTOT_U(1)*VINF_A(1)
     &                     + CLTOT_U(2)*VINF_A(2)
     &                     + CLTOT_U(3)*VINF_A(3) + CLTOT_A
           VSYS(IV,IVBETA) = CLTOT_U(1)*VINF_B(1)
     &                     + CLTOT_U(2)*VINF_B(2)
     &                     + CLTOT_U(3)*VINF_B(3)
           VSYS(IV,IVROTX) = CLTOT_U(4)
           VSYS(IV,IVROTY) = CLTOT_U(5)
           VSYS(IV,IVROTZ) = CLTOT_U(6)

C
           DO N = 1, NCONTROL
             NV = IVTOT + N
             VSYS(IV,NV) = CLTOT_D(N)
           ENDDO
C
C------------------------------------
          ELSEIF(IC.EQ.ICCY  ) THEN
           VRES(IV) = CYTOT - CONVAL(IC,IR)
           VSYS(IV,IVALFA) = CYTOT_U(1)*VINF_A(1)
     &                     + CYTOT_U(2)*VINF_A(2)
     &                     + CYTOT_U(3)*VINF_A(3)
           VSYS(IV,IVBETA) = CYTOT_U(1)*VINF_B(1)
     &                     + CYTOT_U(2)*VINF_B(2)
     &                     + CYTOT_U(3)*VINF_B(3)
           VSYS(IV,IVROTX) = CYTOT_U(4)
           VSYS(IV,IVROTY) = CYTOT_U(5)
           VSYS(IV,IVROTZ) = CYTOT_U(6)
C
           DO N = 1, NCONTROL
             NV = IVTOT + N
             VSYS(IV,NV) = CYTOT_D(N)
           ENDDO
C
C------------------------------------
          ELSEIF(IC.EQ.ICMOMX) THEN
           VRES(IV) = (CRTOT*CA + CNTOT*SA)*DIR - CONVAL(IC,IR)
           VSYS(IV,IVALFA) = ( CRTOT_U(1)*VINF_A(1)
     &                        +CRTOT_U(2)*VINF_A(2)
     &                        +CRTOT_U(3)*VINF_A(3))*CA*DIR
     &                     + ( CNTOT_U(1)*VINF_A(1)
     &                        +CNTOT_U(2)*VINF_A(2)
     &                        +CNTOT_U(3)*VINF_A(3))*SA*DIR
     &                     + (CRTOT*CA_A + CNTOT*SA_A)*DIR
           VSYS(IV,IVBETA) = ( CRTOT_U(1)*VINF_B(1)
     &                        +CRTOT_U(2)*VINF_B(2)
     &                        +CRTOT_U(3)*VINF_B(3))*CA*DIR
     &                     + ( CNTOT_U(1)*VINF_B(1)
     &                        +CNTOT_U(2)*VINF_B(2)
     &                        +CNTOT_U(3)*VINF_B(3))*SA*DIR
           VSYS(IV,IVROTX) = (CRTOT_U(4)*CA + CNTOT_U(4)*SA)*DIR
           VSYS(IV,IVROTY) = (CRTOT_U(5)*CA + CNTOT_U(5)*SA)*DIR
           VSYS(IV,IVROTZ) = (CRTOT_U(6)*CA + CNTOT_U(6)*SA)*DIR
C
           DO N = 1, NCONTROL
             NV = IVTOT + N
             VSYS(IV,NV) = (CRTOT_D(N)*CA + CNTOT_D(N)*SA)*DIR
           ENDDO
C
C------------------------------------
          ELSEIF(IC.EQ.ICMOMY) THEN
           VRES(IV) = CMTOT - CONVAL(IC,IR)
           VSYS(IV,IVALFA) = CMTOT_U(1)*VINF_A(1)
     &                     + CMTOT_U(2)*VINF_A(2)
     &                     + CMTOT_U(3)*VINF_A(3)
           VSYS(IV,IVBETA) = CMTOT_U(1)*VINF_B(1)
     &                     + CMTOT_U(2)*VINF_B(2)
     &                     + CMTOT_U(3)*VINF_B(3)
           VSYS(IV,IVROTX) = CMTOT_U(4)
           VSYS(IV,IVROTY) = CMTOT_U(5)
           VSYS(IV,IVROTZ) = CMTOT_U(6)
C
           DO N = 1, NCONTROL
             NV = IVTOT + N
             VSYS(IV,NV) = CMTOT_D(N)
           ENDDO
C
C------------------------------------
          ELSEIF(IC.EQ.ICMOMZ) THEN
           VRES(IV) = (CNTOT*CA - CRTOT*SA)*DIR - CONVAL(IC,IR)
           VSYS(IV,IVALFA) = ( CNTOT_U(1)*VINF_A(1)
     &                        +CNTOT_U(2)*VINF_A(2)
     &                        +CNTOT_U(3)*VINF_A(3))*CA*DIR
     &                     - ( CRTOT_U(1)*VINF_A(1)
     &                        +CRTOT_U(2)*VINF_A(2)
     &                        +CRTOT_U(3)*VINF_A(3))*SA*DIR
     &                     + (CNTOT*CA_A - CRTOT*SA_A)*DIR
           VSYS(IV,IVBETA) = ( CNTOT_U(1)*VINF_B(1)
     &                        +CNTOT_U(2)*VINF_B(2)
     &                        +CNTOT_U(3)*VINF_B(3))*CA*DIR
     &                     - ( CRTOT_U(1)*VINF_B(1)
     &                        +CRTOT_U(2)*VINF_B(2)
     &                        +CRTOT_U(3)*VINF_B(3))*SA*DIR
           VSYS(IV,IVROTX) = (CNTOT_U(4)*CA - CRTOT_U(4)*SA)*DIR
           VSYS(IV,IVROTY) = (CNTOT_U(5)*CA - CRTOT_U(5)*SA)*DIR
           VSYS(IV,IVROTZ) = (CNTOT_U(6)*CA - CRTOT_U(6)*SA)*DIR
C
           DO N = 1, NCONTROL
             NV = IVTOT + N
             VSYS(IV,NV) = (CNTOT_D(N)*CA - CRTOT_D(N)*SA)*DIR
           ENDDO
C
C------------------------------------
          ELSE
           DO N = 1, NCONTROL
             ICCON = ICTOT + N
             IVCON = IVTOT + N
             IF(IC.EQ.ICCON) THEN
              VRES(IV) = DELCON(N) - CONVAL(ICCON,IR)
              VSYS(IV,IVCON) = 1.0
              GO TO 100
             ENDIF
           ENDDO
C
           WRITE(*,*) '? Illegal constraint index: ', IC
          ENDIF
C
 100    CONTINUE
C

c        write(*,*)
c        do k = 1, nvtot
c          write(*,'(1x,40f9.4)') (vsys(k,l), l=1, nvtot), vres(k)
c        enddo
c        write(*,*)
C
C------ LU-factor,  and back-substitute RHS
        CALL LUDCMP(IVMAX,NVTOT,VSYS,IVSYS,WORK)
        CALL BAKSUB(IVMAX,NVTOT,VSYS,IVSYS,VRES)
C
C------ set Newton deltas
        DAL = -VRES(IVALFA)
        DBE = -VRES(IVBETA)
        DWX = -VRES(IVROTX)
        DWY = -VRES(IVROTY)
        DWZ = -VRES(IVROTZ)
        DO N = 1, NCONTROL
          IV = IVTOT + N
          DDC(N) = -VRES(IV)
        ENDDO
C
        IF(INFO .GE. 1) THEN
C------- display Newton deltas
         IF(ITER.EQ.1) THEN
          WRITE(*,*)
          WRITE(*,1902) 'iter',
     &            ' d(alpha)  ',
     &            ' d(beta)   ',
     &            ' d(pb/2V)  ',
     &            ' d(qc/2V)  ',
     &            ' d(rb/2V)  ',
     &            (DNAME(K), K=1, NCONTROL)
 1902     FORMAT(1X,A4,5A11,1X,30A11)
         ENDIF
         WRITE(*,1905) ITER, 
     &                 DAL/DTR, DBE/DTR, 
     &                 DWX*BREF/2.0, DWY*CREF/2.0, DWZ*BREF/2.0,
     &                 (DDC(K), K=1, NCONTROL)
 1905    FORMAT(1X,I3,40E11.3)
        ENDIF
C
C------ limits on angles and rates
        DMAXA = DMAX
        DMAXR = 5.0*DMAX/BREF
C
C------ if changes are too big, configuration is probably untrimmable
        IF(ABS(ALFA+DAL).GT.DMAXA) THEN
         WRITE(*,*) 'Cannot trim.  Alpha too large.  a =',(ALFA+DAL)/DTR
         RETURN
        ENDIF
C
        IF(ABS(BETA+DBE).GT.DMAXA) THEN
         WRITE(*,*) 'Cannot trim.  Beta too large.  b =',(BETA+DBE)/DTR
         RETURN
        ENDIF
C
        IF(ABS(WROT(1)+DWX).GT.DMAXR) THEN
         WRITE(*,*) 'Cannot trim.  Roll rate too large.  pb/2V =', 
     &               (WROT(1)+DWX)*BREF*0.5
         RETURN
        ENDIF
C
        IF(ABS(WROT(2)+DWY).GT.DMAXR) THEN
         WRITE(*,*) 'Cannot trim.  Pitch rate too large.  qc/2V =',
     &               (WROT(2)+DWY)*CREF*0.5
         RETURN
        ENDIF
C
        IF(ABS(WROT(3)+DWZ).GT.DMAXR) THEN
         WRITE(*,*) 'Cannot trim.  Yaw rate too large.  rb/2V =',
     &               (WROT(3)+DWZ)*BREF*0.5
         RETURN
        ENDIF
C
C------ update
        ALFA  = ALFA  + DAL
        BETA  = BETA  + DBE
        WROT(1) = WROT(1) + DWX
        WROT(2) = WROT(2) + DWY
        WROT(3) = WROT(3) + DWZ
        DO K = 1, NCONTROL
          DELCON(K) = DELCON(K) + DDC(K)
        ENDDO
C
C
C------ set VINF() vector from new ALFA,BETA
        CALL VINFAB
C
        IF(NCONTROL.GT.0) THEN
C------- set new GAM_D
         CALL GDCALC(NCONTROL,LCONDEF,ENC_D,GAM_D)
        ENDIF
C
        IF(NDESIGN.GT.0) THEN
C------- set new GAM_G
         CALL GDCALC(NDESIGN ,LDESDEF,ENC_G,GAM_G)
        ENDIF
C
C------ sum AIC matrices to get GAM,SRC,DBL
        CALL GAMSUM
C
C------ sum AIC matrices to get WC,WV
        CALL VELSUM
C
C------ compute forces
        CALL AERO
C
C
C------ convergence check
        DELMAX = MAX( ABS(DAL), 
     &                ABS(DBE),
     &                ABS(DWX*BREF/2.0),
     &                ABS(DWY*CREF/2.0),
     &                ABS(DWZ*BREF/2.0) )
        DO K = 1, NCONTROL
          DELMAX = MAX( DELMAX , ABS(DDC(K)) )
        ENDDO
C
        IF(DELMAX.LT.EPS) THEN
         LSOL = .TRUE.
C------- mark trim case as being converged
         ITRIM(IR) = IABS(ITRIM(IR))
         GO TO 191
        ENDIF
C
 190  CONTINUE
      IF(NITER.GT.0) THEN
       WRITE(*,*) 'Trim convergence failed'
       LSOL = .FALSE.
       RETURN
      ENDIF
C
 191  CONTINUE
      PARVAL(IPALFA,IR) = ALFA/DTR
      PARVAL(IPBETA,IR) = BETA/DTR
      PARVAL(IPROTX,IR) = WROT(1)*0.5*BREF
      PARVAL(IPROTY,IR) = WROT(2)*0.5*CREF
      PARVAL(IPROTZ,IR) = WROT(3)*0.5*BREF
      PARVAL(IPCL  ,IR) = CLTOT
C
      LSEN = .TRUE.
      RETURN
C
      END ! EXEC



      SUBROUTINE OPTGET
C-------------------------------------------------
C     Allows toggling and setting of various 
C     printing and plotting stuff.
C-------------------------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*4 ITEMC
      CHARACTER*80 COMARG
      CHARACTER*50 SATYPE, ROTTYPE
      LOGICAL ERROR
C
      REAL    RINPUT(20)
      INTEGER IINPUT(20)
      LOGICAL LINPUT(20)
C
 1000 FORMAT(A)
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
 100  CONTINUE
      IF(LSA_RATES) THEN
        ROTTYPE =
     &         'Rates,moments about Stability Axes, X along Vinf'
      ELSE
        ROTTYPE =
     &         'Rates,moments about Body Axes, X along geometric X axis'
      ENDIF
C  
      WRITE(*,1110) LPTOT,LPSURF,LPSTRP,LPELE,
     &              LPHINGE,LPDERIV,
     &              LTRFORCE,LVISC,LBFORCE,
     &                SATYPE,ROTTYPE,IZSYM,ZSYM,SAXFR,VRCORE
 1110   FORMAT(/'   ======================================'
     &         /'    P rint default output for...'
     &         /'        total     :  ',L2,
     &         /'        surfaces  :  ',L2,
     &         /'        strips    :  ',L2,
     &         /'        elements  :  ',L2,
     &        //'    H inge mom. output:  ',L2,
     &         /'    D erivative output:  ',L2,
     &        //'    T rail.leg forces:  ',L2,
     &         /'    V iscous forces  :  ',L2,
     &         /'    B ody forces     :  ',L2,
     &        //'    A xis orient. :  ', A, 
     &         /'    R ate,mom axes:  ', A,
     &         /'    Z  symmetry   :  ',I2,' @ Z =',F10.4
     &         /'    S pan axis x/c:  ',F10.4
     &         /'    C ore/ds ratio:  ',F10.4)
C
C
      CALL ASKC(' ..Select item to change^',ITEMC,COMARG)
C
C------------------------------------------------------
      IF    (ITEMC.EQ.'    ') THEN
        RETURN
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'A   ') THEN
        LNASA_SA = .NOT.LNASA_SA
        CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'R   ') THEN
        LSA_RATES = .NOT.LSA_RATES
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'V   ') THEN
        LVISC = .NOT.LVISC
        IF(LVISC) THEN
          WRITE(*,*) 'Forces will include profile drag'
         ELSE
          WRITE(*,*) 'Forces will not include profile drag'
        ENDIF
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'B   ') THEN
        LBFORCE = .NOT.LBFORCE
        IF(LBFORCE) THEN
          WRITE(*,*) 'Forces will include body forces'
         ELSE
          WRITE(*,*) 'Forces will not include body forces'
        ENDIF
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'T   ') THEN
        LTRFORCE = .NOT.LTRFORCE
        IF(LTRFORCE) THEN
          WRITE(*,*) 'Forces on trailing legs will be included'
         ELSE
          WRITE(*,*) 'Forces on trailing legs will not be included'
        ENDIF
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'P   ') THEN
 128   IF(COMARG(1:1).NE.' ') THEN
         NINP = 4
         CALL GETLGV(COMARG,LINPUT,NINP,ERROR)
         IF(ERROR) GO TO 130
C
         IF(NINP.GE.1) LPTOT   = LINPUT(1)
         IF(NINP.GE.2) LPSURF  = LINPUT(2)
         IF(NINP.GE.3) LPSTRP  = LINPUT(3)
         IF(NINP.GE.4) LPELE   = LINPUT(4)
C
         GO TO 100
        ENDIF
C
 130    WRITE(*,2100)
     &  'Enter print flags T/F (total,surf,strip,elem)'
 2100   FORMAT(1X,A,': ', $)
C
        READ(*,1000) COMARG
        IF(COMARG.EQ.' ') THEN
         GO TO 100
        ELSE
         GO TO 128
        ENDIF
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'H   ') THEN
        LPHINGE = .NOT.LPHINGE
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'D   ') THEN
        LPDERIV = .NOT.LPDERIV
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'Z   ') THEN
       WRITE(*,*) ' '
       WRITE(*,*) 'Currently:'
       IF(IZSYM.EQ.0) THEN
         WRITE (*,1015)
        ELSEIF(IZSYM.GT.0) THEN
         WRITE (*,1016) ZSYM
        ELSEIF(IZSYM.LT.0) THEN
         WRITE (*,1017) ZSYM
       ENDIF
       WRITE(*,*) 'Enter symmetry flag: -1 Free surface'
       WRITE(*,*) '                      0 no Z symmetry'
       WRITE(*,*) '                      1 Ground plane'
       ZSYMIN = 0.0
       READ(*,*,ERR=100) IZSYMIN
       IF(IZSYMIN.NE.0.0) THEN
         WRITE(*,2100) 'Enter Z for symmetry plane'
         READ(*,*,ERR=100) ZSYMIN
       ENDIF
       IZSYM = IZSYMIN
       ZSYM  = ZSYMIN
       LAIC = .FALSE.
       LSRD = .FALSE.
       LSOL = .FALSE.
       LVEL = .FALSE.
       LSEN = .FALSE.
C
 1015  FORMAT(' Z Symmetry: No symmetry assumed')
 1016  FORMAT(' Z Symmetry: Ground plane at Zsym =',F10.4)
 1017  FORMAT(' Z Symmetry: Free surface at Zsym =',F10.4)
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'S   ') THEN
        NINP = 1
        CALL GETFLT(COMARG,RINPUT,NINP,ERROR)
C
        IF(ERROR .OR. NINP.LE.0) THEN
         RINPUT(1) = SAXFR
         WRITE(*,1030) RINPUT(1)
 1030    FORMAT(/' Enter x/c location of spanwise ref. axis:', F10.4)
         CALL READR(1,RINPUT(1),ERROR)
         IF(ERROR) GO TO 100
        ENDIF
C
        SAXFR = MAX( 0.0 , MIN(1.0,RINPUT(1)) )
        CALL ENCALC
        CALL AERO
C
C---------------------------------
      ELSEIF(ITEMC.EQ.'C   ') THEN
        NINP = 1
        CALL GETFLT(COMARG,RINPUT,NINP,ERROR)
C
        IF(ERROR .OR. NINP.LE.0) THEN
         RINPUT(1) = VRCORE
         WRITE(*,1040) RINPUT(1)
 1040    FORMAT(/' Enter core/vortex-strip width:', F10.4)
         CALL READR(1,RINPUT,ERROR)
         IF(ERROR) GO TO 100
        ENDIF
C
        VRCORE = MAX( 0.0 , MIN(1.0,RINPUT(1)) )
        CALL ENCALC
        LAIC = .FALSE.
        LSRD = .FALSE.
        LSOL = .FALSE.
        LSEN = .FALSE.
C
C---------------------------------
      ELSE
        WRITE(*,*) 'Item not recognized'
        GO TO 100
      ENDIF
      GO TO 100
C
      END ! OPTGET



      SUBROUTINE CFRAC(IRUN,NRUN,CPR,NPR)
      CHARACTER*(*) CPR
C
      IZERO = ICHAR('0')
      ITEN = IRUN/10
      IONE = IRUN - 10*(IRUN/10)
      IF(ITEN.LE.0) THEN
       CPR = CHAR(IZERO+IONE) // '/'
      ELSE
       CPR = CHAR(IZERO+ITEN) // CHAR(IZERO+IONE) // '/'
      ENDIF
C
      NPR = INDEX(CPR,'/')
      ITEN = NRUN/10
      IONE = NRUN - 10*(NRUN/10)
      IF(ITEN.LE.0) THEN
       CPR = CPR(1:NPR)
     &    // CHAR(IZERO+IONE) // '^'
      ELSE
       CPR = CPR(1:NPR)
     &    // CHAR(IZERO+ITEN) // CHAR(IZERO+IONE) // '^'
      ENDIF
C
      NPR = INDEX(CPR,'^') - 1
C
      RETURN
      END ! CFRAC



      SUBROUTINE RCOPY(IRSET,IR)
      INCLUDE 'AVL.INC'
C
      DO IV = 1, NVTOT
        ICON(IV,IRSET) = ICON(IV,IR)
      ENDDO
      DO IC = 1, NCTOT
        CONVAL(IC,IRSET) = CONVAL(IC,IR)
      ENDDO
      DO IP = 1, NPTOT
        PARVAL(IP,IRSET) = PARVAL(IP,IR)
      ENDDO
C
      RTITLE(IRSET) = RTITLE(IR)
      ITRIM(IRSET) = ITRIM(IR)
      NEIGEN(IRSET) = NEIGEN(IR)
C
      DO KE = 1, JEMAX
        EVAL(KE,IRSET) = EVAL(KE,IR)
        DO JE = 1, JEMAX
          EVEC(KE,JE,IRSET) = EVEC(KE,JE,IR)
        ENDDO
      ENDDO
C
      RETURN
      END ! RCOPY





      SUBROUTINE GETFILE(LU,FNAME)
      CHARACTER(*) FNAME
C
      CHARACTER*1 ANS, DUMMY
C
 1000 FORMAT(A)
C
      IF(FNAME.EQ.' ') THEN
       CALL ASKS('Enter filename, or <return> for screen output^',FNAME)
      ENDIF
C
      IF(FNAME.EQ.' ') THEN
       LU = 6
       RETURN
C
      ELSE
       LU = 11
       OPEN(LU,FILE=FNAME,STATUS='OLD',FORM='FORMATTED',ERR=44)
       WRITE(*,*) 
       WRITE(*,*) 'File exists.  Append/Overwrite/Cancel  (A/O/C)?  C'
       READ(*,1000) ANS
       IF    (INDEX('Aa',ANS).NE.0) THEN
 40     CONTINUE
        READ(LU,1000,END=42) DUMMY
        GO TO 40
 42     CONTINUE
       ELSEIF(INDEX('Oo',ANS).NE.0) THEN
        REWIND(LU) 
       ELSE
        CLOSE(LU)
        LU = 0
       ENDIF
       RETURN
C
 44    OPEN(LU,FILE=FNAME,STATUS='UNKNOWN',FORM='FORMATTED',ERR=48)
       REWIND(LU)
       RETURN
C
 48    CONTINUE
       LU = -1
       RETURN
      ENDIF
      END ! GETFILE

