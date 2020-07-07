C***********************************************************************
C    Module:  plutil.f
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
 
      SUBROUTINE XYPLOT(N,X,Y,XOFF,XSF,YOFF,YSF,ILIN,SH,ISYM)
      DIMENSION X(N), Y(N)
C
      IF(ISYM.LE.0) CALL XYLINE(N,X,Y,XOFF,XSF,YOFF,YSF,ILIN)
      IF(ISYM.NE.0) CALL XYSYMB(N,X,Y,XOFF,XSF,YOFF,YSF,SH,IABS(ISYM))
C
      RETURN
      END



      SUBROUTINE XAXIS2(X1,Y1,XAXT,DXANN,FANN,DANN,IFLAG,CHT,NDIG)
C
C---XAXIS2 differs from libPlt XAXIS by having a flag to suppress both
C   end annotations rather than the zero annotation.
C.......................................................
C     X1,Y1  starting point of x axis
C     XAXT   length of x axis 
C     DXANN  distance between annotations
C     FANN   first annotation value
C     DANN   delta annotation value
C     IFLAG  flag to suppress end annotations 
C            = 0    all annotations
C            = 1    suppress first annotation
C            = 2    suppress last  annotation
C            = 3    suppress first and last annotations
C     CHT    character height   ( - = annotation above axis)
C     NDIG   number of digits to right of decimal point
C            = -1   no decimal point
C            = -2   number of digits determined internally
C.......................................................
C
      XAX = ABS(XAXT)
      IF(XAX.LE.0) RETURN
      CH = ABS(CHT)
C
C--- determine # of digits to use for annotations
      IF(NDIG.LE.-2) THEN
        ND = 1 - MAX( 0 , INT(LOG10(DANN)) )
        IF(DANN*10**ND - AINT(DANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
        IF(DANN*10**ND - AINT(DANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
      ELSE
        ND = NDIG
      ENDIF
C
C---- x-axis
      CALL PLOT(X1,Y1,3)
      CALL PLOT(X1+XAX,Y1,2)
      NANN = 1 + IFIX(XAX/DXANN + 0.1)
C
C---- annotate x-axis
      DO 10 NT=1, NANN
        XT = X1 + DXANN*FLOAT(NT-1)
C---- skip annotation for first or last annotation position as given by IFLG
        IF(MOD(IFLAG,2).EQ.1 .AND. NT.EQ.1)    GO TO 10
        IF(IFLAG.GT.1        .AND. NT.EQ.NANN) GO TO 10
C
        CALL PLOT(XT,Y1-0.2*CH,3)
        CALL PLOT(XT,Y1+0.2*CH,2)
        RN = FANN + DANN*FLOAT(NT-1)
        GRN = 0.
        IF(RN.NE.0.0) GRN = ALOG10(ABS(RN)+0.5/10.0**ND)
        GRN = MAX(GRN,0.0)
        NABC = INT(GRN) + 2 + ND
        WIDTH = 0.95*CH*FLOAT(NABC)
        IF(RN.LT.0.0) WIDTH = WIDTH + CH
        XNUM = XT - 0.5*WIDTH
        YNUM = Y1 - 2.1*CH
        IF(CHT.LT.0.0) YNUM = Y1 + 0.9*CH
C
        CALL PLNUMB(XNUM,YNUM,CH,RN,0.0,ND)
   10 CONTINUE
C
      RETURN
      END ! XAXIS
 



      SUBROUTINE VIEWINIT(AZIM, ELEV, TILT, ROBINV)
C----------------------------------------------------------------------------
C     Sets up projection for points in 3-D Cartesian space
C     onto a 2-D plane from the viewpoint of an observer
C     at a specified location.  This can be used to "view" 
C     a 3-D object described by a set of points on a planar
C     2-D graphics screen. 
C        The viewing  plane, which has its own x,y coordinate
C     system, always faces the observer but can be turned
C     around the viewing axis, thus simulating the observer
C     tilting his head while looking at the object.  This tilt
C     is specified by giving a vector which "points up" relative
C     to the observer.
C        The distance of the observer from the object is specified
C     explicitly.  This does not affect much the size of the viewed
C     object, since the viewing plane contains the 3-D space origin
C     and hence is at or near the object.  It does however affect the
C     apparent distortion of the object due to perspective.  This
C     is very useful to convey the 3-dimensionality of the object.
C     If the observer is very very far away, there is no distortion
C     (as in a mechanical drawing).
C
C     AZIM      azimuth   angle of observer in degrees   (input)
C     ELEV      elevation angle of observer in degrees   (input)
C     TILT      tilt      angle of observer in degress   (input)
C     ROBINV    1/(distance to observer)                 (input)
C
C     XYZOB(.)  Cartesian vector pointing towards observer  (internal output)
C               (magnitude irrelevant)
C     XYZUP(.)  Cartesian vector which points "up" from the 
C               observer's viewpoint (magnitude irrelevant) (internal output)
C----------------------------------------------------------------------------
C
      COMMON /VIEWDATA/ RINV,
     &       XIHAT, YIHAT, ZIHAT,
     &       XJHAT, YJHAT, ZJHAT,
     &       XKHAT, YKHAT, ZKHAT

      REAL XYZOB(3), XYZUP(3)
C
      DATA DTR / 0.01745329 /
C
      XYZOB(1) = -COS(DTR*AZIM) * COS(DTR*ELEV)
      XYZOB(2) =  SIN(DTR*AZIM) * COS(DTR*ELEV)
      XYZOB(3) =                  SIN(DTR*ELEV)
C
      XYZUP(1) =  0.
      XYZUP(2) = -SIN(DTR*TILT)
      XYZUP(3) =  COS(DTR*TILT)
C
C---- unit view vector out of viewing plane (towards observer)
      XKHAT = XYZOB(1)/SQRT(XYZOB(1)**2 + XYZOB(2)**2 + XYZOB(3)**2)
      YKHAT = XYZOB(2)/SQRT(XYZOB(1)**2 + XYZOB(2)**2 + XYZOB(3)**2)
      ZKHAT = XYZOB(3)/SQRT(XYZOB(1)**2 + XYZOB(2)**2 + XYZOB(3)**2)
C
C---- vector along plane's local x coordinate: (up vector)X(view vector)
      XIP = XYZUP(2)*ZKHAT - XYZUP(3)*YKHAT
      YIP = XYZUP(3)*XKHAT - XYZUP(1)*ZKHAT
      ZIP = XYZUP(1)*YKHAT - XYZUP(2)*XKHAT
C
C---- normalize plane's x coordinate vector
      XIHAT = XIP/SQRT(XIP**2 + YIP**2 + ZIP**2)
      YIHAT = YIP/SQRT(XIP**2 + YIP**2 + ZIP**2)
      ZIHAT = ZIP/SQRT(XIP**2 + YIP**2 + ZIP**2)
C
C---- unit vector along plane's y coord.: (view vector)X(x unit vector)
      XJHAT = YKHAT*ZIHAT - ZKHAT*YIHAT
      YJHAT = ZKHAT*XIHAT - XKHAT*ZIHAT
      ZJHAT = XKHAT*YIHAT - YKHAT*XIHAT
C
      RINV = ROBINV
C
      RETURN
      END ! VIEWINIT



      SUBROUTINE VIEWPROJ(XYZ,N,XYZPROJ)
      REAL XYZ(3,N), XYZPROJ(3,N)
C.......................................................................
C
C     Projects one or more points in 3-D Cartesian space
C     onto a 2-D plane from the viewpoint of an observer
C     at a specified location using the projection transformation
C     setup in VIEWINIT.
C
C     This can be used to "view" a 3-D object described by a set 
C     of points on a planar 2-D graphics screen.  This routine also 
C     returns the out-of-plane vector for depth comparison.
C        The viewing  plane, which has its own x,y coordinate
C     system, always faces the observer but can be turned
C     around the viewing axis, thus simulating the observer
C     tilting his head while looking at the object.  This tilt
C     is specified by giving a vector which "points up" relative
C     to the observer.
C        The distance of the observer from the object is specified
C     explicitly.  This does not affect much the size of the viewed
C     object, since the viewing plane contains the 3-D space origin
C     and hence is at or near the object.  It does however affect the
C     apparent distortion of the object due to perspective.  This
C     is very useful to convey the 3-dimensionality of the object.
C     If the observer is very very far away, there is no distortion
C     (as in a mechanical drawing).
C
C     XYZ(.)       Cartesian point coordinates                  (input)
C     N            number of points                             (input)
C     XYZPROJ(.)   projected point coordinates on viewing plane (output)
C.......................................................................
C
      COMMON /VIEWDATA/ RINV,
     &       XIHAT, YIHAT, ZIHAT,
     &       XJHAT, YJHAT, ZJHAT,
     &       XKHAT, YKHAT, ZKHAT
C
C
C---- go over all points
      DO I=1, N
        RDOTI = XYZ(1,I)*XIHAT + XYZ(2,I)*YIHAT + XYZ(3,I)*ZIHAT
        RDOTJ = XYZ(1,I)*XJHAT + XYZ(2,I)*YJHAT + XYZ(3,I)*ZJHAT
        RDOTK = XYZ(1,I)*XKHAT + XYZ(2,I)*YKHAT + XYZ(3,I)*ZKHAT
C
C------ viewing-axis component of vector
        RKX = RDOTK*XKHAT
        RKY = RDOTK*YKHAT
        RKZ = RDOTK*ZKHAT
C
C------ projected vector scaling factor due to perspective
        VSCAL = 1.0 / SQRT( (XKHAT-RINV*RKX)**2
     &                    + (YKHAT-RINV*RKY)**2
     &                    + (ZKHAT-RINV*RKZ)**2 )
C
C------ dot vector into plane coordinate system unit vectors, and scale
        XYZPROJ(1,I) = VSCAL * RDOTI
        XYZPROJ(2,I) = VSCAL * RDOTJ
        XYZPROJ(3,I) = VSCAL * RDOTK
      END DO
C
      RETURN
      END



      SUBROUTINE OPLSET(IDEV,IDEVH,IPSLU,LSVMOV,
     &                  SIZE,PAR,
     &                  XMARG,YMARG,XPAGE,YPAGE,
     &                  CSIZE,SCRNFR,LCURS,LCREV)
      LOGICAL LSVMOV,LCURS,LCREV
C-----------------------------------------------------------
C     Allows user modification of various plot parameters.
C-----------------------------------------------------------
      CHARACTER*1 VAR
      CHARACTER*4 COMAND
      CHARACTER*128 COMARG
      CHARACTER*10 CHCURS, CHLAND
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR, LGRAPH, LCOLOR, LFILES
C
 1000 FORMAT(A)
C
 1    CONTINUE
      IF(LCURS) THEN
       CHCURS = 'Cursor    '
      ELSE
       CHCURS = 'Keyboard  '
      ENDIF
C
      IF(SCRNFR.GT.0.0) THEN
       CHLAND = 'Landscape '
      ELSE
       CHLAND = 'Portrait  '
      ENDIF
C
      ASF = ABS(SCRNFR)
C
      LGRAPH = IDEV  .GE. 1
      LCOLOR = IDEVH .EQ. 4
      LFILES = IPSLU .LT. 0
C
      WRITE(*,2000) LGRAPH, LCOLOR, LFILES, LSVMOV,
     &              PAR, ASF, SIZE,
     &              XPAGE,YPAGE, XMARG,YMARG, 
     &              CSIZE,
     &              CHLAND, CHCURS
 2000 FORMAT(' ...............................................'
     &     //'  G raphics-enable flag        ', L2,
     &      /'  C olor PostScript output?    ', L2,
     &      /'  I ndividual PS file output?  ', L2,
     &      /'  H ardcopy movie PS output?   ', L2,
     &     //'  A spect ratio of plot object ', F8.4
     &      /'  W indow/screen size fraction ', F8.4
     &     //'  S ize of plot object         ', F6.2,'"'
     &      /'  P age dimensions             ', F6.2,' x',F6.2,'"'
     &      /'  M argins from page edges     ', F6.2,'",',F6.2,'"'
     &      /'  F ont size (relative)        ', F8.4
     &     //'  O rientation of plot:        ', A 
     &      /'  B lowup input method:        ', A )
ccc     &      /'  R everse-video output?       ', L2 )
C
C   A B C D E F G H I J K L M N O P Q R S T U V W X Y Z
C   x x x     x x           x   x x     x       x     
C
 5    CALL ASKC('      Option, Value   (or <Return>) ^',COMAND,COMARG)
C
      DO I=1, 20
        IINPUT(I) = 0.0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 0
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 0
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
      VAR = COMAND(1:1)
      IF (VAR.EQ.'0' .OR. VAR.EQ.' ') THEN
       RETURN
C
      ELSEIF (INDEX('Gg',VAR).NE.0) THEN
        IF(IDEV.EQ.0) THEN
         IDEV = 1
        ELSE
         IDEV = 0
        ENDIF
C
      ELSEIF (INDEX('Cc',VAR).NE.0) THEN
        IF(IDEVH.EQ.2) THEN
         IDEVH = 4
        ELSE
         IDEVH = 2
        ENDIF
C
      ELSEIF (INDEX('Ii',VAR).NE.0) THEN
        IF(IPSLU.EQ.0) THEN
         IPSLU = -1
        ELSE
         IPSLU = 0
        ENDIF
C
      ELSEIF (INDEX('Ss',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          SIZE = RINPUT(1)
        ELSE
          CALL ASKR('Enter size (in)^',SIZE)
        ENDIF
C
      ELSEIF (INDEX('Aa',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          PAR = RINPUT(1)
        ELSE
          CALL ASKR('Enter aspect ratio^',PAR)
        ENDIF
C
      ELSEIF (INDEX('Pp',VAR).NE.0) THEN
        IF(NINPUT.GE.2) THEN
          XPAGE = RINPUT(1)
          YPAGE = RINPUT(2)
        ELSEIF(NINPUT.GE.1) THEN
          XPAGE = RINPUT(1)
          CALL ASKR('Enter page Y dimension (in)^',YPAGE)
        ELSE
          CALL ASKR('Enter page X dimension (in)^',XPAGE)
          CALL ASKR('Enter page Y dimension (in)^',YPAGE)
        ENDIF
C
      ELSEIF (INDEX('Mm',VAR).NE.0) THEN
        IF(NINPUT.GE.2) THEN
          XMARG = RINPUT(1)
          YMARG = RINPUT(2)
        ELSEIF(NINPUT.GE.1) THEN
          XMARG = RINPUT(1)
          CALL ASKR('Enter page Y margin (in)^',YMARG)
        ELSE
          CALL ASKR('Enter page X margin (in)^',XMARG)
          CALL ASKR('Enter page Y margin (in)^',YMARG)
        ENDIF
C
      ELSEIF (INDEX('Ff',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          CSIZE = RINPUT(1)
        ELSE
          CALL ASKR('Enter character font size^',CSIZE)
        ENDIF
C
      ELSEIF (INDEX('Ww',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          ASF = RINPUT(1)
        ELSE
          CALL ASKR('Enter window/screen size fraction^',ASF)
        ENDIF
        SCRNFR = SIGN( ASF , SCRNFR )
C
      ELSEIF (INDEX('Bb',VAR).NE.0) THEN
        LCURS = .NOT. LCURS
C
      ELSEIF (INDEX('Oo',VAR).NE.0) THEN
        SCRNFR = -SCRNFR
        WRITE(*,*)
        WRITE(*,*) 'Swapping X,Y page dimensions'
        XTMP = XPAGE
        YTMP = YPAGE
        XPAGE = YTMP
        YPAGE = XTMP
C
      ELSEIF (INDEX('Rr',VAR).NE.0) THEN
        LCREV = .NOT. LCREV
C
      ELSE
        WRITE(*,*) '*** Item not recognized ***'
      ENDIF
      GO TO 1
C
      END ! OPLSET





      subroutine AXISADJ2(xmin,xmax,xspan,deltax,ntics)
C...Make scaled axes with engineering increments between tics
C
C   Input:    xmin, xmax - input range for which scaled axis is desired
C
C   Output:   xmin, xmax - adjusted range for scaled axis
C             xspan      - adjusted span of scaled axis
C             deltax     - increment to be used for scaled axis
C             ntics      - number of axis tics (each deltax long)
C
      real    xmin,xmax,xspan,deltax,xinc,xinctbl(4)
      integer ntics,i
      data    xinctbl / 0.1, 0.2, 0.5, 1.0 /
c
      xspan1 = xmax-xmin
      if (xspan1.eq.0.) xspan1 = 1.
c
      xpon = ifix(log10(xspan1))
      xspan = xspan1 / 10.**xpon
c
      do i = 1, 4
        xinc = xinctbl(i)
        ntics = 1 + ifix(xspan/xinc)
        if (ntics.LE.12) go to 1
      end do
c
   1  deltax = xinc*10.**xpon
      xmin = deltax*  ifloor(xmin/deltax)
      xmax = deltax*iceiling(xmax/deltax)
      xspan = xmax - xmin
      return
      end
