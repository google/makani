C***********************************************************************
C    Module:  aplotmd.f
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

      SUBROUTINE PLOTMD(AZIM, ELEV, TILT, RINV, KEIG, IR)
C
C----Plotting module for AVL vortex lattice program
C
C    Plots geometry and loading results for vortex lattice
C     Geometry plots:
C       Surfaces     (straight tapered panels of chordwise strips of vortices)
C       Strips       (chordwise strips of vortices)
C       Vortex legs  (bound vortex legs)
C       Control pts  (vortex element control points)
C       Camber slope (camber of each strip - used for establishing BC's)
C       Hinge lines  (surface deflection axis and deflected surface outline)
C       Strip loading (chordwise plot of vortex loading on each strip)
C
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      LOGICAL LMOVE, LEVIEW, LCPAN
      LOGICAL LINITVIEW, LFIRST
      SAVE    LINITVIEW, LCPAN
      LOGICAL ERROR
      CHARACTER*4 OPT
      CHARACTER*1 CHKEY
C
      REAL*8 TCPU0, TCPU1, TCPU
C
      REAL ANG0(3), POS0(3)
      REAL ANGE(3), POSE(3), VELE(3), ROTE(3)
      REAL ANG(3), POS(3), ANGP(3),DANG(3)
      REAL TT(3,3), TT_ANG(3,3,3),
     &     RT(3,3), RT_ANG(3,3,3)
C
      REAL EVR(JEMAX)
C
      REAL RINP(10)
C
C---- viewpoint changes (deg), zoom/unzoom, perspective scale factors
      DATA DAZIM, DELEV, ZUFAC, DPAN , PRAT 
     &     / 5.0 , 5.0 , 1.25 , 0.075, 1.1 /
C
C---- phase step and scale factor step for interactive phase plots
      DATA DPHASE, SCALEF / 5.0 , 1.25 /
C
      CALL GETWINSIZE(XWIND,YWIND)
      CCH = 0.75*CH
      XPLT0 = XABS2USR(PMARG)
      YPLT0 = YABS2USR(YWIND-YMARG-PMARG) - 1.2*CCH
C
      YSP = 2.0
C
C---- Initialization for plot program variables
      IF(LPLTNEW .OR. (.NOT.LPLOT)) THEN 
        LPLTNEW = .FALSE.
        LINITVIEW = .FALSE.
      ENDIF
C
      LFIRST = .TRUE.
C
C---- default is plot at zero baseline Euler angles
      LEVIEW = .TRUE.
C
C---- default is camera pans with aircraft
      LCPAN = .TRUE.
C
C---- no movie mode
      LMOVE = .FALSE.
C
C---- find geometry limits
      CALL GLIMS(GMIN,GMAX,.FALSE.)
C
C---- set stuff for this mode
      VEE = PARVAL(IPVEE,IR)
      REFL = BREF*UNITL
      REFV = VEE
C
      EVMIN = (REFV/REFL) * 1.0E-5
      CALL EVNORM(EVEC(1,KEIG,IR),ESF,REFL,REFV)
C
      SIGMA = REAL(EVAL(KEIG,IR))
      OMEGA = IMAG(EVAL(KEIG,IR))
      EVMAG = SQRT(SIGMA**2 + OMEGA**2)
C
      FRQ = OMEGA / (2.0*PI)
      IF(SIGMA .EQ. 0.0) THEN
       DAMPR = 0.
      ELSE
       DAMPR = -SIGMA / EVMAG
      ENDIF
C
C---- initial position
      POS0(1) = 0.
      POS0(2) = 0.
      POS0(3) = 0.
C
      ANG0(1) = PARVAL(IPPHI,IR)*DTR
      ANG0(2) = PARVAL(IPTHE,IR)*DTR
      ANG0(3) = PARVAL(IPPSI,IR)*DTR
C
      XYZREF(1) = PARVAL(IPXCG,IR)
      XYZREF(2) = PARVAL(IPYCG,IR)
      XYZREF(3) = PARVAL(IPZCG,IR)
C
C---- sat baseline velocities and rotation rates
      ALFA = PARVAL(IPALFA,IR)*DTR
      BETA = PARVAL(IPBETA,IR)*DTR
      CALL VINFAB
      WROT(1) = PARVAL(IPROTX,IR)*2.0/BREF
      WROT(2) = PARVAL(IPROTY,IR)*2.0/CREF
      WROT(3) = PARVAL(IPROTZ,IR)*2.0/BREF
C
C
C---- time step sign
      TSIGN = 1.0
C
C---- initial time
      TIMED = 0.
C
      POS(1) = POS0(1)
      POS(2) = POS0(2)
      POS(3) = POS0(3)
      ANG(1) = ANG0(1)
      ANG(2) = ANG0(2)
      ANG(3) = ANG0(3)
C
C---- set time step based on max Euler angle change
      WRMAX = MAX( ABS(WROT(1)) , ABS(WROT(2)) , ABS(WROT(3)) )
C
C***************************************************
    1 CONTINUE
      WRITE(*,2041) LCPAN, LMWAIT
 2041 FORMAT(/'  ------------------------------'
     &       /'   L eft           R ight       '
     &       /'   U p             D own        '
     &       /'   C lear                       '
     &      //'   Z oom           N ormal size '
     &       /'   I ngress        O utgress    '
     &       /'   H ardcopy       A nnotate    '
     &      //'   P anning camera toggle: ', L1
     &       /'   W ait-for-clock toggle: ', L1
     &      //'  < > 0  mode play              '
     &       /'  - + 1  mode scale             '
     &       /'  S      mode sign change       '
     &       /'  { }    slower-mo / faster-mo  '
     &      //' Type in plot window:  Command,  or  <space> to exit')
C
C   A B C D E F G H I J K L M N O P Q R S T U V W X Y Z
c   x   x x       x x     x   x x x   x x   x   x     x
C
C***************************************************
C---- Setup view transformation 
    4 CALL VIEWINIT(AZIM, ELEV, TILT, RINV)
C
C---- scale and project x,y,z orientation arrows UAXARW(....)
C-    also project x,y,z unit vectors UAXDIR(..)
      ARWLEN = 0.2*BREF
C
      RHEADP = RHEAD*ARWLEN
      DO IXYZ = 1, 3
        DO K = 1, 3
          DO IL = 1, NLINAX
            UAXARWP(K,1,IL,IXYZ) = UAXARW(K,1,IL,IXYZ)*ARWLEN
            UAXARWP(K,2,IL,IXYZ) = UAXARW(K,2,IL,IXYZ)*ARWLEN
          ENDDO
        ENDDO
      ENDDO
      NLIN = 3 * 2*NLINAX
      NAXD = 3   
      CALL VIEWPROJ(UAXARWP,NLIN,UAXARWP)
      CALL VIEWPROJ(UAXDIR ,NAXD,UAXDIRP)
C
      DO IAXD = 1, NAXD
        DIRMAG = SQRT( UAXDIRP(1,IAXD)**2
     &               + UAXDIRP(2,IAXD)**2
     &               + UAXDIRP(3,IAXD)**2 )
        IF(DIRMAG .GT. 0.0) THEN
         UAXDIRP(1,IAXD) = UAXDIRP(1,IAXD)/DIRMAG
         UAXDIRP(2,IAXD) = UAXDIRP(2,IAXD)/DIRMAG
         UAXDIRP(3,IAXD) = UAXDIRP(3,IAXD)/DIRMAG
        ENDIF
      ENDDO
C
C***************************************************
C---- compute perturbed position at current or new time
    6 CONTINUE
C
      IF(LMOVE) THEN
C----- advance or retreat in time
       TIMED = TIMED + TSIGN*DTIMED*TMOFAC
C
       IF(LCPAN) THEN
C------ leave aircraft position unchanged (since camera is following it)

       ELSE
C------ integrate velocities and angular rates to next movie frame time
C
C------ time interval in AVL units
        DTIME = DTIMED*VEE/UNITL * TMOFAC
C
C------ set number of time steps for suitable accuracy (limit max angle change)
        DAMAX = WRMAX*DTIME
        NTIME = MAX( 1 , INT(DAMAX/0.02) )
C
C------ integrate over time interval  t = TIME..TIME+DTIME
        DO ITIME = 1, NTIME
          DT = TSIGN * DTIME/FLOAT(NTIME)
C
C-------- predictor step, slopes evaluated at  t
          CALL RATEKI3(ANG,RT,RT_ANG)
          DO K = 1, 3
            DANG(K) = DT*( RT(K,1)*WROT(1)
     &                   + RT(K,2)*WROT(2)
     &                   + RT(K,3)*WROT(3) )
            ANGP(K) = ANG(K) + DANG(K)
          ENDDO
C
C-------- corrector step, slopes evaluated at  t + dt
          CALL RATEKI3(ANGP,RT,RT_ANG)
          DO K = 1, 3
            DANGP = DT*( RT(K,1)*WROT(1)
     &                 + RT(K,2)*WROT(2)
     &                 + RT(K,3)*WROT(3) )
C---------- midpoint angles at  t + dt/2
            ANGP(K) = ANG(K) + 0.25*(DANG(K) + DANGP)
          ENDDO
C
C-------- use midpoint-angle matrices
          CALL RATEKI3(ANGP,RT,RT_ANG)
          CALL ROTENS3(ANGP,TT,TT_ANG)
C
C-------- final integration step, using midpoint slopes
          DO K = 1, 3
            POS(K) = POS(K) - DT*( TT(K,1)*VINF(1)
     &                           + TT(K,2)*VINF(2)
     &                           + TT(K,3)*VINF(3) )
            ANG(K) = ANG(K) + DT*( RT(K,1)*WROT(1)
     &                           + RT(K,2)*WROT(2)
     &                           + RT(K,3)*WROT(3) )
          ENDDO
        ENDDO
       ENDIF
      ENDIF
C
      EPHASE = OMEGA*TIMED / DTR
      EFAC = ESF*EIGENF
      CALL EVREAL(EVEC(1,KEIG,IR),EVAL(KEIG,IR), EFAC,TIMED, EVR)
C
C
C---- scale from standard (SI or English) to AVL units
C
      EVR(JEX) = EVR(JEX)/UNITL
      EVR(JEY) = EVR(JEY)/UNITL
      EVR(JEZ) = EVR(JEZ)/UNITL
C
      EVR(JEU) = EVR(JEU)/VEE
      EVR(JEV) = EVR(JEV)/VEE
      EVR(JEW) = EVR(JEW)/VEE
C
      EVR(JEP) = EVR(JEP)*UNITL/VEE
      EVR(JEQ) = EVR(JEQ)*UNITL/VEE
      EVR(JER) = EVR(JER)*UNITL/VEE
C
CCC   EVR(JEPH) = EVR(JEPH)
CCC   EVR(JETH) = EVR(JETH)
CCC   EVR(JEPS) = EVR(JEPS)
C
c      write(*,*)
c      write(*,*) 'xyz', evr(jex),evr(jey),evr(jez)
c      write(*,*) 'uvw', evr(jeu),evr(jev),evr(jew)
c      write(*,*) 'pqr', evr(jep),evr(jeq),evr(jer)
c      write(*,*) 'reh', evr(jeph),evr(jeth),evr(jeps)
C
C
C---- set final position, including eigenvector perturbation
      POSE(1) = POS(1) + EVR(JEX)
      POSE(2) = POS(2) + EVR(JEY)
      POSE(3) = POS(3) + EVR(JEZ)
C
C---- set final angles, including eigenvector perturbation
      ANGE(1) = ANG(1) + EVR(JEPH)
      ANGE(2) = ANG(2) + EVR(JETH)
      ANGE(3) = ANG(3) + EVR(JEPS)
C
      VELE(1) = VINF(1) - EVR(JEU)
      VELE(2) = VINF(2) - EVR(JEV)
      VELE(3) = VINF(3) - EVR(JEW)
C
      ROTE(1) = WROT(1) + EVR(JEP)
      ROTE(2) = WROT(2) + EVR(JEQ)
      ROTE(3) = WROT(3) + EVR(JER)
C
      VMAGE = SQRT(VELE(1)**2 + VELE(2)**2 + VELE(3)**2)
      V13   = SQRT(VELE(1)**2              + VELE(3)**2)
      ALFAE = ATAN2(  VELE(3) , VELE(1) )
      BETAE = ATAN2( -VELE(2) , V13     )
C
c      CALL RATEKI3(ANG,RT,RT_ANG)
c      DO K = 1, 3
c        DANG(K) =    RT(K,1)*WROT(1)
c     &             + RT(K,2)*WROT(2)
c     &             + RT(K,3)*WROT(3)
c      ENDDO
c      write(1,1234) ( pos(k)/12.0, k=1, 3),
c     &              ( ang(k)/dtr , k=1, 3),
c     &              ( dang(k)*bref/dtr    , k=1, 3)
c      write(2,1234) (pose(k)/12.0, k=1, 3), (ange(k)/dtr, k=1, 3)
c 1234 format(1x,9f10.3)
C
C
C---- set limits for baseline position
      CALL ROTENS3(ANG,TT,TT_ANG)
      CALL GRLIMS(VMINP,VMAXP,.TRUE. ,TT ,XYZREF,POS )
      IF(LFIRST) THEN
C----- first frame... set plot limits directly
       XMIN = VMINP(1)
       YMIN = VMINP(2)
       XMAX = VMAXP(1)
       YMAX = VMAXP(2)
      ELSE
C----- clip to new limits
       XMIN = MIN(XMIN,VMINP(1))
       YMIN = MIN(YMIN,VMINP(2))
       XMAX = MAX(XMAX,VMAXP(1))
       YMAX = MAX(YMAX,VMAXP(2))
      ENDIF
      LFIRST = .FALSE.
C
C
C---- also enforce limits for perturbed position
      CALL ROTENS3(ANGE,TT,TT_ANG)
      CALL GRLIMS(VMINP,VMAXP,.TRUE. ,TT ,XYZREF,POSE)
      XMIN = MIN(XMIN,VMINP(1))
      YMIN = MIN(YMIN,VMINP(2))
      XMAX = MAX(XMAX,VMAXP(1))
      YMAX = MAX(YMAX,VMAXP(2))
C
      CALL OFFINI
C
C***************************************************
C---- plot the baseline and displaced geometry
    8 CONTINUE
      CALL PVLINI(TITLE,AZIM,ELEV,TILT,VERSION,LSVMOV)
C
      XPLT = XPLT0
      YPLT = YPLT0
      CALL PLCHAR(XPLT,YPLT,CCH,'Run  ',0.0,5)
      CALL PLNUMB(999.,YPLT,CCH,FLOAT(IR),0.0,-1)
C
      YPLT = YPLT - YSP*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'Mode ',0.0,5)
      CALL PLNUMB(999.,YPLT,CCH,FLOAT(KEIG),0.0,-1)
C
      YPLT = YPLT - YSP*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'f = ',0.0,4)
      CALL PLNUMB(999.,YPLT,CCH,FRQ,0.0,4)
      CALL PLCHAR(999.,YPLT,CCH,' cycles/' ,0.0,8)
      CALL PLCHAR(999.,YPLT,CCH,UNCHT(1:NUT),0.0,NUT)
C
      YPLT = YPLT - YSP*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'z = ',0.0,4)
      CALL PLNUMB(999.,YPLT,CCH,DAMPR,0.0,6)
C
      YPLT = YPLT - YSP*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'t = ',0.0,4)
      CALL PLNUMB(999.,YPLT,CCH,TIMED ,0.0,2)
      CALL PLCHAR(999.,YPLT,CCH,UNCHT(1:NUT),0.0,NUT)
C
      YPLT = YPLT - YSP*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'w = ',0.0, 4)
      CALL PLCHAR(XPLT,YPLT,CCH,' t  ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,EPHASE,0.0,-1)
      CALL PLMATH(999.,YPLT,CCH,   '"',0.0, 1)
C
      YPLT = YPLT - 1.0*CCH
C
      YPLT = YPLT - YSP*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'x = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,POSE(1)*UNITL,0.0,-1)
      CALL PLCHAR(999.,YPLT,CCH,UNCHL,0.0,NUL)
C
      YPLT = YPLT - YSP*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'y = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,POSE(2)*UNITL,0.0,-1)
      CALL PLCHAR(999.,YPLT,CCH,UNCHL,0.0,NUL)
C
      YPLT = YPLT - YSP*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'z = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,POSE(3)*UNITL,0.0,-1)
      CALL PLCHAR(999.,YPLT,CCH,UNCHL,0.0,NUL)
C
      YPLT = YPLT - 1.0*CCH
C
      YPLT = YPLT - YSP*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'f = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,ANGE(1)/DTR,0.0,1)
      CALL PLMATH(999.,YPLT,CCH,'"',0.0,1)
C
      YPLT = YPLT - YSP*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'q = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,ANGE(2)/DTR,0.0,1)
      CALL PLMATH(999.,YPLT,CCH,'"',0.0,1)
C
      YPLT = YPLT - YSP*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'y = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,ANGE(3)/DTR,0.0,1)
      CALL PLMATH(999.,YPLT,CCH,'"',0.0,1)
C
      YPLT = YPLT - 1.0*CCH
C
      YPLT = YPLT - YSP*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'V = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,VMAGE*VEE  ,0.0,-1)
      CALL PLCHAR(999.,YPLT,CCH,UNCHV,0.0,NUV)
C
      YPLT = YPLT - YSP*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'a = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,ALFAE/DTR,0.0,2)
      CALL PLMATH(999.,YPLT,CCH,'"',0.0,1)
C
      YPLT = YPLT - YSP*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'b = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,BETAE/DTR,0.0,2)
      CALL PLMATH(999.,YPLT,CCH,'"',0.0,1)
C
C
C---- Setup hidden line data
      CALL HIDINITE(.TRUE., ANGE,POSE,XYZREF)
C
C---- plot baseline position
      CALL GETCOLOR(ICOL0)
      CALL PLOTMODE(ANG ,POS ,XYZREF,0)

C---- plot mode-displaced position
      CALL NEWCOLORNAME('RED')
      CALL PLOTMODE(ANGE,POSE,XYZREF,1)

      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
      IF(LMOVE .AND. LMWAIT) THEN
C----- waste time if in movie mode and waiting is enabled
       CALL SECONDS(TCPU1)
 9     CONTINUE
       CALL SECONDS(TCPU)
       DTCPU = TCPU - TCPU0
ccc       write(*,*) tcpu0, tcpu, dtcpu, dtimed
       IF(DTCPU .LT. DTIMED) GO TO 9
C
c       dtcpu1 = tcpu1 - tcpu0
c       WRITE(*,*) 'plot time frac:', DTCPU1/DTIMED
      ENDIF
C
ccc      CALL DRAWTOSCREEN
C---------------------------------------------
C---- set user command key
 10   CONTINUE
C
      CHKEY = ' '
      CALL GETCURSORXY(XX,YY,CHKEY)
C
      LMOVE = .FALSE.
      IF(CHKEY .EQ. ' ') THEN
        RETURN
C
C---- change observer's azimuth and elevation angles
      ELSEIF(INDEX('Ll',CHKEY).NE.0) THEN
        AZIM = AZIM + DAZIM
        IF(AZIM .GT.  180.01) AZIM = AZIM - 360.0
        GO TO 4
C
      ELSEIF(INDEX('Rr',CHKEY).NE.0) THEN
        AZIM = AZIM - DAZIM
        IF(AZIM .LT. -180.01) AZIM = AZIM + 360.0
        GO TO 4
C
      ELSEIF(INDEX('Uu',CHKEY).NE.0) THEN
        ELEV = ELEV + DELEV
        IF(ELEV .GT.  180.01) ELEV = ELEV - 360.0
        GO TO 4
C
      ELSEIF(INDEX('Dd',CHKEY).NE.0) THEN
        ELEV = ELEV - DELEV
        IF(ELEV .LT. -180.01) ELEV = ELEV + 360.0
        GO TO 4
C
      ELSEIF(INDEX('Cc',CHKEY).NE.0) THEN
        ELEV = 0.
        AZIM = 0.
        LFIRST = .TRUE.
        GO TO 4
C
      ELSEIF(INDEX('Zz',CHKEY).NE.0) THEN
C------ put center at current cursor location, increase magnification
        XOFF = XX/SF + XOFF
        YOFF = YY/SF + YOFF
        SF = SF*ZUFAC
        XDIF =    1.0/SF
        YDIF = PLOTAR/SF
        XOFF = XOFF - 0.5*XDIF
        YOFF = YOFF - 0.5*YDIF
        GO TO 8
C
      ELSEIF(INDEX('Nn',CHKEY).NE.0) THEN
C------ force resetting of scale,offset factors
        LFIRST = .TRUE.
        GO TO 6
C
      ELSEIF(INDEX('Ii',CHKEY).NE.0) THEN
C------ increase perspective distortion
        IF(RINV.EQ.0.0) THEN
         RINV = 0.02/BREF
        ELSE
         RINV = RINV*PRAT
        ENDIF
        WRITE(*,8010) 1.0/RINV
        GO TO 4
C
      ELSEIF(INDEX('Oo',CHKEY).NE.0) THEN
C------ decrease perspective distortion
        IF(RINV .LT. 0.02/BREF) THEN
         RINV = 0.0
         WRITE(*,*) '  Observer distance = infinity  (no perspective)'
        ELSE
         RINV = RINV/PRAT
         WRITE(*,8010) 1.0/RINV
        ENDIF
        GO TO 4
C
      ELSEIF(INDEX('Aa',CHKEY).NE.0) THEN
C------ annotate plot
        CALL NEWPEN(2)
        CALL ANNOT(CH)
        GO TO 1
C
      ELSEIF(INDEX('Hh',CHKEY).NE.0) THEN
C------ write PostScript version of current screen
        CALL REPLOT(IDEVH)
        GO TO 1
C
c      ELSEIF(INDEX('Ee',CHKEY).NE.0) THEN
cC------ toggle viewing at true/zero Euler angles
c        LEVIEW = .NOT.LEVIEW
c        IF(LEVIEW) THEN
c         WRITE(*,*) 'View at actual Euler angles'
c        ELSE
c         WRITE(*,*) 'View at zero Euler angles'
c        ENDIF
c        GO TO 6
C
      ELSEIF(INDEX('Pp',CHKEY).NE.0) THEN
C------ toggle camera panning
        LCPAN = .NOT.LCPAN
        IF(LCPAN) THEN
         WRITE(*,*) 'Camera will pan with aircraft'
        ELSE
         WRITE(*,*) 'Camera will not pan with aircraft'
        ENDIF
        EPHASE = 0.
        TIMED = 0.
        POS(1) = POS0(1)
        POS(2) = POS0(2)
        POS(3) = POS0(3)
        ANG(1) = ANG0(1)
        ANG(2) = ANG0(2)
        ANG(3) = ANG0(3)
        EIGENF = 1.
        LFIRST = .TRUE.
        GO TO 6
C
      ELSEIF(INDEX('Ww',CHKEY).NE.0) THEN
C------ toggle clock waiting
        LMWAIT = .NOT.LMWAIT
        IF(LMWAIT) THEN
         WRITE(*,*) 'Frame plotting will wait for clock'
        ELSE
         WRITE(*,*) 'Frame plotting will not wait for clock'
        ENDIF
        GO TO 6
C
      ELSEIF(INDEX('0' ,CHKEY).NE.0) THEN
C------ set zero phase
ccc        EPHASE = 0.0
        TIMED = 0.
        POS(1) = POS0(1)
        POS(2) = POS0(2)
        POS(3) = POS0(3)
        ANG(1) = ANG0(1)
        ANG(2) = ANG0(2)
        ANG(3) = ANG0(3)
        GO TO 6
C
      ELSEIF(INDEX('1' ,CHKEY).NE.0) THEN
C------ set unity scale factor
        EIGENF = 1.0
        GO TO 6
C
      ELSEIF(INDEX('<,',CHKEY).NE.0) THEN
C------ decrease phase by DPHASE amount
ccc        EPHASE = EPHASE - DPHASE
        LMOVE = .TRUE.
        TSIGN = -1.0
        CALL SECONDS(TCPU0)
        GO TO 6
C
      ELSEIF(INDEX('>.',CHKEY).NE.0) THEN
C------ increase phase by DPHASE amount
ccc        EPHASE = EPHASE + DPHASE
        LMOVE = .TRUE.
        TSIGN = +1.0
        CALL SECONDS(TCPU0)
        GO TO 6
C
      ELSEIF(INDEX('-_',CHKEY).NE.0) THEN
C------ decrease response vector coefficient
        EIGENF = EIGENF / SCALEF
        GO TO 6
C
      ELSEIF(INDEX('+=',CHKEY).NE.0) THEN
C------ increase response vector coefficient
        EIGENF = EIGENF * SCALEF
        GO TO 6
C
      ELSEIF(INDEX('{[',CHKEY).NE.0) THEN
C------ slow down time advance
        TMOFAC = TMOFAC / SCALEF
        IF(ABS(TMOFAC-1.0) .LT. 0.01) THEN
         TMOFAC = 1.0
         WRITE(*,8020) TMOFAC, ' (real time)'
        ELSE
         WRITE(*,8020) TMOFAC
        ENDIF
        GO TO 6
C
      ELSEIF(INDEX('}]',CHKEY).NE.0) THEN
C------ speed up time advance
        TMOFAC = TMOFAC * SCALEF
        IF(ABS(TMOFAC-1.0) .LT. 0.01) THEN
         TMOFAC = 1.0
         WRITE(*,8020) TMOFAC, ' (real time)'
        ELSE
         WRITE(*,8020) TMOFAC
        ENDIF
        GO TO 6
C
      ELSEIF(INDEX('Ss',CHKEY).NE.0) THEN
C------ change mode sign
        EIGENF = -EIGENF
        GO TO 6
C
      ELSE
        WRITE(*,*) '* Key command not recognized'
        GO TO 1
C
      ENDIF
 8010 FORMAT('  Observer distance =', F11.3,1X,A)
 8020 FORMAT('  Time advance factor =', F11.3,1X,A)
C
      END ! PLOTMD




      SUBROUTINE EVREAL(EVEC,EVAL,EFAC,TIMED,EVR)
      COMPLEX EVEC(*), EVAL
      REAL EVR(*)
      INCLUDE 'AINDEX.INC'
C
      COMPLEX EVFAC
C
C
C---- set complex mode scale factor
      EVFAC = CEXP( EVAL*TIMED ) * EFAC
C
C---- real part of phsyically-scaled scaled eigenmode
      NJ = JETOT
      DO J = 1, NJ
        EVR(J) = REAL( EVFAC * EVEC(J) )
      ENDDO
C
      END ! EVREAL



      SUBROUTINE EVNORM(EV,ESF,REFL,REFV)
      COMPLEX EV(*)
C
      INCLUDE 'AINDEX.INC'
C
      DATA EFRAC / 0.5 /
C
      REFW = REFV/REFL
      REFA = 0.5
C
      RMAX = 0.
C
C---- test x,y,z changes
      RMAX = MAX( RMAX , CABS( EV(JEX)/REFL ) )
      RMAX = MAX( RMAX , CABS( EV(JEY)/REFL ) )
      RMAX = MAX( RMAX , CABS( EV(JEZ)/REFL ) )
C
C---- test u,v,w changes
      RMAX = MAX( RMAX , CABS( EV(JEU)/REFV ) )
      RMAX = MAX( RMAX , CABS( EV(JEV)/REFV ) )
      RMAX = MAX( RMAX , CABS( EV(JEW)/REFV ) )
C
C---- test p,q,r changes
      RMAX = MAX( RMAX , CABS( EV(JEP)/REFW ) )
      RMAX = MAX( RMAX , CABS( EV(JEQ)/REFW ) )
      RMAX = MAX( RMAX , CABS( EV(JER)/REFW ) )
C
C---- test Euler angle changes
      RMAX = MAX( RMAX , CABS( EV(JEPH)/REFA ) )
      RMAX = MAX( RMAX , CABS( EV(JETH)/REFA ) )
      RMAX = MAX( RMAX , CABS( EV(JEPS)/REFA ) )
C
      IF(RMAX .EQ. 0.0) THEN
        ESF = 1.0
      ELSE
        ESF = EFRAC/RMAX
      ENDIF
C
      RETURN
      END ! EVNORM



      SUBROUTINE PLOTMODE(ANG,POS,XYZR,IPAT)
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      REAL ANG(3), POS(3), XYZR(3)
C
      REAL TT(3,3), TT_ANG(3,3,3)
C
      REAL PTS_LINES(3,2,NVMAX), ID_LINES(NVMAX),
     &     PTS_LPROJ(3,2,NVMAX)
C
      DATA ID_LINES / NVMAX * 0.0 /

      INCLUDE 'MASKS.INC'
C
      IF(IPAT.EQ.0) THEN
       LMOUT = LMASK2
       LMCHD = LMASk3
       IPOUT = 2
       IPCHD = 1
      ELSE
       LMOUT = LMASK0
       LMCHD = LMASK1
       IPOUT = 4
       IPCHD = 1
      ENDIF
C
C---- number of line segments for drawing a body section "hoop"
      KK = 18
C
      CALL ROTENS3(ANG,TT,TT_ANG)
C
C---- go over surfaces
      DO 100 N = 1, NSURF
C
C------ do only surfaces which are to be plotted
        IF(.NOT.LPLTSURF(N)) GO TO 100
C
        J1 = JFRST(N)
        JN = JFRST(N) + NJ(N)-1
        JINC = 1
        IF(IMAGS(N).LT.0) THEN
         J1 = JFRST(N) + NJ(N)-1
         JN = JFRST(N)
         JINC = -1
        ENDIF
C
C------ Chordlines plotted
        IF(LCHORDLINE) THEN
         IP = 0
         DO J = J1, JN, JINC
           IP = IP+1
           PTS_LINES(1,1,IP) = RLE1(1,J)
           PTS_LINES(2,1,IP) = RLE1(2,J)
           PTS_LINES(3,1,IP) = RLE1(3,J)
           PTS_LINES(1,2,IP) = RLE1(1,J) + CHORD1(J)
           PTS_LINES(2,2,IP) = RLE1(2,J)
           PTS_LINES(3,2,IP) = RLE1(3,J)
           ID_LINES(IP) = J
           CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
           CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
         END DO
         IP = IP+1
         PTS_LINES(1,1,IP) = RLE2(1,JN)
         PTS_LINES(2,1,IP) = RLE2(2,JN)
         PTS_LINES(3,1,IP) = RLE2(3,JN)
         PTS_LINES(1,2,IP) = RLE2(1,JN) + CHORD2(JN)
         PTS_LINES(2,2,IP) = RLE2(2,JN)
         PTS_LINES(3,2,IP) = RLE2(3,JN)
         ID_LINES(IP) = JN
         CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
         CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
C
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C
         CALL NEWPAT(LMCHD)
         CALL NEWPEN(IPCHD)
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
c         DO IP = 1, NLINES
c           CALL PLOT(PTS_LINES(1,1,IP),PTS_LINES(2,1,IP),3)
c           CALL PLOT(PTS_LINES(1,2,IP),PTS_LINES(2,2,IP),2)
c         ENDDO
        ENDIF
C
C
C--- Surface boundary lines plotted
        IP = 0
        IP = IP+1
        PTS_LINES(1,1,IP) = RLE1(1,J1)
        PTS_LINES(2,1,IP) = RLE1(2,J1)
        PTS_LINES(3,1,IP) = RLE1(3,J1)
        PTS_LINES(1,2,IP) = RLE1(1,J1) + CHORD1(J1)
        PTS_LINES(2,2,IP) = RLE1(2,J1)
        PTS_LINES(3,2,IP) = RLE1(3,J1)
        ID_LINES(IP) = J1
        CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
        CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
C
        IP = IP+1
        PTS_LINES(1,1,IP) = RLE2(1,JN)
        PTS_LINES(2,1,IP) = RLE2(2,JN)
        PTS_LINES(3,1,IP) = RLE2(3,JN)
        PTS_LINES(1,2,IP) = RLE2(1,JN) + CHORD2(JN)
        PTS_LINES(2,2,IP) = RLE2(2,JN)
        PTS_LINES(3,2,IP) = RLE2(3,JN)
        ID_LINES(IP) = JN
        CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
        CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
C
        DO J = J1, JN, JINC
          IP = IP+1
          PTS_LINES(1,1,IP) = RLE1(1,J)
          PTS_LINES(2,1,IP) = RLE1(2,J)
          PTS_LINES(3,1,IP) = RLE1(3,J)
          PTS_LINES(1,2,IP) = RLE2(1,J)
          PTS_LINES(2,2,IP) = RLE2(2,J)
          PTS_LINES(3,2,IP) = RLE2(3,J)
          ID_LINES(IP) = J
          CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
          CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
C
          IP = IP+1
          PTS_LINES(1,1,IP) = RLE1(1,J) + CHORD1(J)
          PTS_LINES(2,1,IP) = RLE1(2,J)
          PTS_LINES(3,1,IP) = RLE1(3,J)
          PTS_LINES(1,2,IP) = RLE2(1,J) + CHORD2(J)
          PTS_LINES(2,2,IP) = RLE2(2,J)
          PTS_LINES(3,2,IP) = RLE2(3,J)
          ID_LINES(IP) = J
          CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
          CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
        END DO
C
        NLINES = IP
        NPROJ = 2*NLINES
        CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C
        CALL NEWPAT(LMOUT)
        CALL NEWPEN(IPOUT)
        CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
c        DO IP = 1, NLINES
c          CALL PLOT(PTS_LINES(1,1,IP),PTS_LINES(2,1,IP),3)
c          CALL PLOT(PTS_LINES(1,2,IP),PTS_LINES(2,2,IP),2)
c        ENDDO
C
 100  CONTINUE
C
C
C---- go over bodies
      DO 300 N = 1, NBODY
C
C------ do only bodies which are to be plotted
        IF(.NOT.LPLTBODY(N)) GO TO 300
C
        L1 = LFRST(N)
        LN = LFRST(N) + NL(N)-1
        LINC = 1
C
C------ source lines plotted
        IP = 0
        DO L = L1, LN-LINC, LINC
          IP = IP+1
          PTS_LINES(1,1,IP) = RL(1,L)
          PTS_LINES(2,1,IP) = RL(2,L)
          PTS_LINES(3,1,IP) = RL(3,L)
          PTS_LINES(1,2,IP) = RL(1,L+LINC)
          PTS_LINES(2,2,IP) = RL(2,L+LINC)
          PTS_LINES(3,2,IP) = RL(3,L+LINC)
          CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
          CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
        END DO
C
        NLINES = IP
        NPROJ = 2*NLINES
        CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C
        CALL NEWPAT(LMCHD)
        CALL NEWPEN(IPCHD)
        CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
c        DO IP = 1, NLINES
c          CALL PLOT(PTS_LINES(1,1,IP),PTS_LINES(2,1,IP),3)
c          CALL PLOT(PTS_LINES(1,2,IP),PTS_LINES(2,2,IP),2)
c        ENDDO
C
C
C------ Body hoops plotted
ccc        IF(LCHORDLINE) THEN
         IP = 0
         DO L = L1, LN, LINC
C--------- set "horizontal" and "vertical" body section unit vectors
           IF(LINC.GT.0) THEN
            LM = MAX(L-1,L1)
            LP = MIN(L+1,LN)
           ELSE
            LM = MAX(L-1,LN)
            LP = MIN(L+1,L1)
           ENDIF
           DXL = RL(1,LP) - RL(1,LM)
           DYL = RL(2,LP) - RL(2,LM)
           DZL = RL(3,LP) - RL(3,LM)
           DSL = SQRT(DXL**2 + DYL**2 + DZL**2)
           IF(DSL.NE.0.0) THEN
            DXL = DXL/DSL
            DYL = DYL/DSL
            DZL = DZL/DSL
           ENDIF
C     
           UHX = -DYL
           UHY =  DXL
           UHZ = 0.
C
           UVX = DYL*UHZ - DZL*UHY
           UVY = DZL*UHX - DXL*UHZ
           UVZ = DXL*UHY - DYL*UHX
C
           DO K = 1, KK
             THK1 = 2.0*PI*FLOAT(K-1)/FLOAT(KK)
             THK2 = 2.0*PI*FLOAT(K  )/FLOAT(KK)
C
             HX1 = UHX*COS(THK1)
             HY1 = UHY*COS(THK1)
             HZ1 = UHZ*COS(THK1)
C
             VX1 = UVX*SIN(THK1)
             VY1 = UVY*SIN(THK1)
             VZ1 = UVZ*SIN(THK1)
C
             HX2 = UHX*COS(THK2)
             HY2 = UHY*COS(THK2)
             HZ2 = UHZ*COS(THK2)
C
             VX2 = UVX*SIN(THK2)
             VY2 = UVY*SIN(THK2)
             VZ2 = UVZ*SIN(THK2)
C
             IP = IP+1
             PTS_LINES(1,1,IP) = RL(1,L) + HX1*RADL(L) + VX1*RADL(L)
             PTS_LINES(2,1,IP) = RL(2,L) + HY1*RADL(L) + VY1*RADL(L)
             PTS_LINES(3,1,IP) = RL(3,L) + HZ1*RADL(L) + VZ1*RADL(L)
             PTS_LINES(1,2,IP) = RL(1,L) + HX2*RADL(L) + VX2*RADL(L)
             PTS_LINES(2,2,IP) = RL(2,L) + HY2*RADL(L) + VY2*RADL(L)
             PTS_LINES(3,2,IP) = RL(3,L) + HZ2*RADL(L) + VZ2*RADL(L)
             CALL TETRAN(PTS_LINES(1,1,IP),TT,XYZR,POS)
             CALL TETRAN(PTS_LINES(1,2,IP),TT,XYZR,POS)
           ENDDO
         END DO
C
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C
         CALL NEWPAT(LMOUT)
         CALL NEWPEN(IPOUT)
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
c         DO IP = 1, NLINES
c           CALL PLOT(PTS_LINES(1,1,IP),PTS_LINES(2,1,IP),3)
c           CALL PLOT(PTS_LINES(1,2,IP),PTS_LINES(2,2,IP),2)
c         ENDDO
ccc        ENDIF
C
 300  CONTINUE
C
      CALL NEWPAT(LMASK0)
      RETURN
      END ! PLOTMODE





      SUBROUTINE PLEMAP(XORG0,YORG0, 
     &                  IRUN1,IRUN2,
     &                  KEVIEW,IRVIEW,
     &                  LPROOT,LPRNUM,OVERLAY,
     &                  TMIN,TMAX,TDEL,TFAC,
     &                  FMIN,FMAX,FDEL,FFAC )
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      LOGICAL LPROOT(JEMAX,*), LPRNUM(JEMAX,*), OVERLAY
C
      LOGICAL LCONVT(NRMAX), LLINE, LBASE
      LOGICAL LPGRID
C
      REAL PARV(IPMAX,NRMAX)
C
      CS  = 0.55*CH
      CSL = 0.70*CH
      CSN = 0.75*CH
      CSP = 0.55*CH

      DXLAB = 2.0*CSP
      DYLAB = 2.3*CSP
C
C---- data root overlay symbol size and type
      CSS = 0.55*CS
      ISYMB = 3
C
      LPGRID = .TRUE.
C
C---- initialize plot window and move origin away from lower left corner
      CALL PLTINI(IDEV)
C
      CALL PLOT(1.0,0.75,-3)
      CALL NEWFACTOR(SIZE)
C
      CALL GETCOLOR(ICOL0)
C
C---- aspect ratio, lower-left plot location
      XORG = XORG0
      YORG = YORG0
C
C---- draw plot
      PARXY = PLOTAR - DYLAB*(1.3 + FLOAT(IRUN2-IRUN1+1))
      CALL PLROOT(XORG,YORG, 
     &            IRUN1, IRUN2, IRCOLOR, 
     &            JEMAX,NEIGEN,EVAL,LPROOT,LPRNUM,
     &            UNCHT(1:NUT), PARXY,CS, LPGRID, 
     &            TMIN,TMAX,TDEL,TFAC,FMIN,FMAX,FDEL,FFAC)
C
cC---- plot "o" over shift location
c      CALL NEWPEN(1)
c      XC1 = XORG + (REAL(ZSHIFT)-TMIN)*TFAC
c      YC1 = YORG + (IMAG(ZSHIFT)-FMIN)*FFAC
c      CALL PLSYMB(XC1,YC1,CS,1,0.0,0)
C
      IF(IRVIEW.GE.1 .AND. IRVIEW.LE.NRUN) THEN
       IR = IRVIEW
       IF(KEVIEW.GT.0 .AND. KEVIEW.LE.NEIGEN(IR)) THEN
C------ plot "+" over currently-examined root, if any
        CALL NEWPEN(1)
        XC1 = XORG + (REAL(EVAL(KEVIEW,IR))-TMIN)*TFAC
        YC1 = YORG + (IMAG(EVAL(KEVIEW,IR))-FMIN)*FFAC
        CALL PLSYMB(XC1,YC1,2.0*CS,3,0.0,0)
       ENDIF
      ENDIF
C
      IF(OVERLAY) THEN
C----- data exists... overlay it
       CALL GETCOLOR(ICOL0)
C
       CALL NEWPEN(2)
       XORG = XORG0
       YORG = YORG0
C
       DO IR = 1, NRMAX
         DO KEIG = 1, NEIGENDAT(IR)
           XEV = REAL(EVALDAT(KEIG,IR))
           YEV = IMAG(EVALDAT(KEIG,IR))
           IF(YEV .GE. -1.0E-4) THEN
            CALL NEWCOLOR(IRCOLOR(IR))
            XPLT = XORG + (XEV-TMIN)*TFAC
            YPLT = YORG + (YEV-FMIN)*FFAC
            CALL PLSYMB(XPLT,YPLT,CSS,ISYMB,0.0,0)
           ENDIF
         ENDDO
       ENDDO
       CALL NEWPEN(1)
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
      CALL PLFLUSH
C
C---- put up parameter list
      DO IR = IRUN1, IRUN2
        DO IP = 1, IPMAX
          PARV(IP,IR) = PARVAL(IP,IR)
          IF(ABS(PARV(IP,IR)) .LT. 1.0E-8) PARV(IP,IR) = 0.
        ENDDO
      ENDDO
C
      XPLT = XORG
      YPLT = YORG + (FMAX-FMIN)*FFAC
C
      CALL PLTPAR(XPLT,YPLT, IRUN1, IRUN2,
     &            PARV, LPPAR,
     &            IRCOLOR, CSP, DXLAB, DYLAB )
C
      YPLT = YPLT + 3.0*CSP
      CALL STRIP(TITLE,NTI)
      CALL NEWPEN(3)
      CALL PLCHAR(XORG,YPLT,CSN,TITLE,0.0,NTI)
C
      CALL PLFLUSH
      RETURN
      END ! PLEMAP



      SUBROUTINE TETRAN(R,TT,RREF,DR)
      REAL R(3), TT(3,3), RREF(3), DR(3)
C
      REAL RB(3)
C
      RB(1) = R(1) - RREF(1)
      RB(2) = R(2) - RREF(2)
      RB(3) = R(3) - RREF(3)
C
      DO K = 1, 3
        R(K) = TT(K,1)*RB(1)
     &       + TT(K,2)*RB(2)
     &       + TT(K,3)*RB(3) + RREF(K) + DR(K)
      ENDDO
C
      RETURN
      END ! TETRAN


