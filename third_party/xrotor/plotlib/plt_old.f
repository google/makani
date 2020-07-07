C***********************************************************************
C    Module:  plt_old.f
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This library is free software; you can redistribute it and/or
C    modify it under the terms of the GNU Library General Public
C    License as published by the Free Software Foundation; either
C    version 2 of the License, or (at your option) any later version.
C
C    This library is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
C    Library General Public License for more details.
C
C    You should have received a copy of the GNU Library General Public
C    License along with this library; if not, write to the Free
C    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C 
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C***********************************************************************
C
C***********************************************************************
C --- Xplot11 "old Versatec-like" routines
C
C     Version 4.46 11/28/01
C
C     Xplot11 interface for older plot routines (similar to that for 
C     Versatec graphics) as a bridge to backwards compatibility with 
C     old programs.
C
C     These routines are not preferred for creating a new application,
C     although they will work.  Use the equivalent routines in 
C     plt_base.f, plt_util.f, etc. if you want more capability.
C
C...Note:
C   If your compiler supports passing character data as integers without
C   complaining and you want old style Versatec SYMBOL compatibility (so 
C   you can call SYMBOL with either characters stuffed into an integer 
C   array or with integer arguments to specify plot symbols) you can use
C   the sections of the following code marked with C+++OldVersatec comments. 
C   In that case you don't need the SYMBL routine, just call SYMBOL with 
C   integer arguments.  You also need to switch marked statements in 
C   SYMBOL, NUMBER and LINE.
C
C***********************************************************************
C
C
      subroutine PLOTS(idummy,ihard,ldev)
C--- Open plotting
C     IHARD - sets hardcopy option  
C             IHARD=>0   hardcopy on
C             IHARD <0   hardcopy off (typically IHARD=-999 sets no hardcopy)
C
C     LDEV  - output device (used to be LDEV=6 for Xwindows)
C             currently LDEV =0 gives BW hardcopy only
C             currently LDEV<>0 gives Xwindow (+ BW    hardcopy if IHARD>0)
C
C    This routine uses IHARD>=0 to select hardcopy output
C    The Xwindow defaults to 2/3 of the root window, landscape mode 
C
      SAVE ifirst, relsize, nunit
      DATA ifirst / 0 /
C
      if(ifirst.EQ.0)  then
        ifirst = 1
        call PLINITIALIZE
        relsize = 0.6666
        nunit = 0
      endif
c
      idev = 0
      if(ldev.NE.0) idev = 1
      if(ihard.GE.0) then
        idev = idev + 2
      endif
cc      write(*,*) 'calling plopen with ',relsize,nunit,idev
      call PLOPEN(relsize,nunit,idev)
c
      return
      end


      subroutine PLOTON
      return
      end


      subroutine PLOTOF
      call PLFLUSH
      return
      end


      subroutine ERASE
      call WINERASE
      return
      end


      subroutine FACTOR(scl)
      call NEWFACTOR(scl)
      return
      end


      subroutine WHERE(xnow,ynow,fact)
C--- Returns last x,y position and scale factor
      call GETLASTXY(xnow,ynow)
      call GETFACTORS(fact,dum)
      return
      end

      subroutine WHEREC(xcurs,ycurs)
      character*1 chkey
      call GETCURSORXY(xcurs,ycurs,chkey)
      return
      end


      subroutine WINPIX(nxpix,nypix,ppi)
C--- Returns Xwindow size in pixels and the pixels per inch of 
C    postsript output (not pixels/inch on the screen!) 
C--- WINPIX has been officially replaced by GETWINSIZEABS in XPLOT11
      include 'pltlib.inc'
      nxpix = GX_SIZ
      nypix = GY_SIZ
      ppi = G_SCALE
      return
      end

      subroutine GETWINPIX(nxpix,nypix,ppi)
C--- Same as WINPIX
      call WINPIX(nxpix,nypix,ppi)
      return
      end



      subroutine GRID (X,Y,NX,XD,NY,YD,LMASK)
      call PLGRID (X,Y,NX,XD,NY,YD,LMASK)
      return
      end



      subroutine COLOR (icol)
C--- Selects a foreground color for defined (old-style) colormap
      common / PLT10_OLD / ncols_old, icolmap_old(0:255)
c
      if(ncols_old.LE.0) then
        write(*,*) '*** COLOR no colormap defined'
      endif
      if(icol.LT.0 .OR. icol.GT.ncols_old) then
        write(*,*) '*** COLOR out of bounds: ',icol
      endif
C--- Use colormap entry
      ic = icolmap_old(icol)
c      write(*,*) 'COLOR ',icol,ic
      call NEWCOLOR(ic)
      return
      end


      subroutine SETCOL (icol,red,grn,blu)
C--- Set up a colormap entry 
C    The "old" color routines simply set up rgb entries in the regular
C    colormap.  They do not use the spectrum colors.
C    Note: repeated calls to SETCOL without reseting the colormap will 
C          (eventually) fill up the allocated 'old-style' colormap.
C
C          To reset the 'old-style' colormap give a color
C          index icol<0, this resets the number of allocated colors to 0.
C
      common / PLT10_OLD / ncols_old, icolmap_old(0:255)
      data ifirst / 0 /
c
      if(ifirst.EQ.0) then
        ifirst = 1
        ncols_old = 0
      endif
c
      if(ncols_old.GE.256) then
        write(*,*) '*** SETCOL no more colors available: ',ncols_old
        return
      endif
c
C--- Reset the colormap if icol<0
      if(ncols_old.LT.0) then
        ncols_old = 0
        return
      endif
c
      ired = ifix(255.*red)
      igrn = ifix(255.*grn)
      iblu = ifix(255.*blu)
C--- Install as a regular colormap color
      call NEWCOLORRGB(ired,igrn,iblu)
      call GETCOLOR(ic)
c      write(*,*) 'SETCOL ',icol,ired,igrn,iblu,ic
      ncols_old = ncols_old + 1
      icolmap_old(icol) = ic
      return
      end


C***********************************************************
C  Versatec-style Utility routines 
C    LINE
C    CURVE
C    AXIS
C    AXIS2
C    SCALE
C    SYMBL  - plots graphics symbol given by integer argument
C    SYMBOL - plots character symbol using stroke graphics that 
C             call the basic PLOT plotting function
C    NUMBER - plots numbers using a format conversion to characters
C***********************************************************



      SUBROUTINE LINE (XARRAY,YARRAY,NPTS,INC,LINTYP,ISYM)
C...Plots array of x,y data as a piecewise linear set of line segments
C
C    Input:   XARRAY   is an array of coordinate points to be joined
C             YARRAY   by smooth curve.
C
C             NPTS     ABS(NPTS) is the number of points to plot from points
C                      in XARRAY and YARRAY.
C                  NPTS<0  indicates that scale factors are located as the
C                       last two elements of each data array (i.e.
C                       NPTS+1 AND NPTS+2).
C                  NPTS>0 indicates that the coordinate points are already
C                       scaled for plotting (no scale factors).
c
C             INC      increment used to access points in XARRAY,YARRAY
C             LINTYP   Plotting flag, ABS(LINTYP) is the symbol plotting 
C                      increment (i.e. +2 gives a symbol at every other point)
C                         LINTYP<0  symbols only
C                         LINTYP=0  lines only
C                         LINTYP>0  lines and symbols
C             ISYM     Symbol index to be used for points with symbols 
C
C  Calls:   PLOT
C
C...Note: 
C   You might need to change the SYMBL call below to SYMBOL if your compiler 
C   supports passing character literals as integers without complaining 
C   and you enable old style Versatec SYMBOL calls with integer arguments.
C   See SYMBOL below...
C
      DIMENSION XARRAY(*),YARRAY(*)
      DATA  SYMSIZE / 0.08 /
C
C...Initialize subscripts
      LMIN = NPTS*INC + 1
      LDX  = LMIN + INC
      NL   = LMIN - INC
C
C...Set limits and scaling factors
      FIRSTX = XARRAY(LMIN)
      DELTAX = XARRAY(LDX)
      FIRSTY = YARRAY(LMIN)
      DELTAY = YARRAY(LDX)
C
C...Current plotting location
      CALL GETLASTXY (XN,YN)
      DF = AMAX1 (ABS ((XARRAY(1) - FIRSTX)/DELTAX - XN),
     &            ABS ((YARRAY(1) - FIRSTY)/DELTAY - YN))
      DL = AMAX1 (ABS ((XARRAY(NL) - FIRSTX)/DELTAX - XN),
     &            ABS ((YARRAY(NL) - FIRSTY)/DELTAY - YN))
      IPEN = 3
      ICODE = -1
      NT = IABS(LINTYP)
C
C...Symbols plotted?
      IF (LINTYP.EQ.0) NT = 1
C
C
      IF (DL.GE.DF) THEN
C...Ascending order
        NF = 1
        NA = NT
        KK = INC
       ELSE
C...Descending order
        NF = NL
        NA = ((NPTS - 1)/NT)*NT + NT - (NPTS - 1)
        KK = -INC
      ENDIF
C
C
      IF(LINTYP.LT.0) THEN
C...Symbols only
        IPENA  = 3
        ICODEA = -1
        LSW    = 1
       ELSE
C...Symbols
        IPENA  = 2
        ICODEA = -2
        LSW    = 0
C...Lines only
        IF(LINTYP.EQ.0)  NA = LDX
      ENDIF
C
C...Plot data
      DO 120 I=1,NPTS
        XN = (XARRAY(NF) - FIRSTX)/DELTAX
        YN = (YARRAY(NF) - FIRSTY)/DELTAY
C...Plot symbol
        IF (NA.EQ.NT) THEN
C---For new style SYMBL call (character string input to SYMBOL)
          CALL SYMBL(XN,YN,SYMSIZE,ISYM,0.0,ICODE)
C+++OldVersatec (integer variable as input to SYMBOL)
C          CALL SYMBL(XN,YN,SYMSIZE,ISYM,0.0,ICODE)
C+++OldVersatec 
          NA = 1
          GO TO 110
        ENDIF
C...Or lines between symbols 
        IF(NA.LT.NT .AND. LSW.NE.0) GO TO 100
C
C...Plot line to new point
        CALL PLOT(XN,YN,IPEN)
C
C...Counters for plotting points and symbols
  100   NA = NA + 1
  110   NF = NF + KK
        ICODE = ICODEA
        IPEN = IPENA
  120 CONTINUE
      CALL PLOTOF
C
      RETURN
      END



      SUBROUTINE CURVE (X,Y,NE,DELTA)
C...Plots curve with solid or dashed lines
C
C    Input:  X,Y  is an array of coordinate points to be joined
C                  by smooth curve.
C
C            NE  ABS(NE) is the number of coordinate points in X and Y.
C              NE<0  indicates that scale factors are located as the
C                    last two elements of each data array (i.e. NE+1 AND NE+2).
C              NE>0  indicates that the coordinate points are already
C                    scaled for plotting (no scale factors).
C
C            DELTA  ABS(DELTA) is the segment length for approximating 
C                   the curve.
C              DELTA<0  indicates that the curve is plotted with dashed lines
C                       of 'delta' length.
C              DELTA>0  indicates that the curve is plotted with a solid line.
C
C  Calls:   PLOT
C
      DIMENSION X(NE),Y(NE)
C
C...Default scale factors
      XOFF = 0.
      YOFF = 0.
      XFAC = 1.
      YFAC = 1.
      NET = NE
      IF(NET.EQ.0) RETURN
C
      IF(NE.LT.0) THEN
C...Scale factors in last two array elements
        NET = -NET
        XOFF = X(NET+1)
        YOFF = Y(NET+1)
        XFAC = X(NET+2)
        YFAC = Y(NET+2)
      ENDIF
C
C...Solid or dashed lines?
      IF(DELTA.EQ.0.0) RETURN
      MPEN = 4
      DELT = DELTA
      IF (DELT.LT.0.0) THEN
        DELT = -DELT
        MPEN = 5
      ENDIF
C
C...Initialize everything
      K = 1
      IPEN = 3
      DLTSQ = DELTA*DELTA
C
C
C...Begin main loop.  
C   (X1,Y1) is joined to (X2,Y2) by arc with direction cosines 
C   (C1,S1) and (C2,S2) at end points.  Final values for previous 
C   arc are initial values for new arc.
C
C...New end point
  110 X2 = (X(K)-XOFF)/XFAC
      Y2 = (Y(K)-YOFF)/YFAC
      IF (K.EQ.NET) GO TO 130
      IF (K.GT.1)   GO TO 140
C
C...First data point (K=1)
  120 IF (NET-2) 122,124,126
  122 X1 = (X(1)-XOFF)/XFAC
      Y1 = (Y(1)-YOFF)/YFAC
      CALL PLOT (X1,Y1,+3)
      GOTO 500
  124 DLTX1 = (X(2)-X(1))/XFAC
      DLTY1 = (Y(2)-Y(1))/YFAC
      DLTX2 = DLTX1
      DLTY2 = DLTY1
      GOTO 128
  126 DLTX1 = (X(2)-X(1))/XFAC
      DLTY1 = (Y(2)-Y(1))/YFAC
      DLTX2 = (X(3)-X(2))/XFAC
      DLTY2 = (Y(3)-Y(2))/YFAC
  128 T1 = DLTX1*DLTX1 + DLTY1*DLTY1
      T2 = DLTX2*DLTX2 + DLTY2*DLTY2
      T3 = 2.*SQRT(T1*T2)
      T1 = -T1
      T2 = T3 + T2
      GO TO 150
C
C...Last data point (K=NET)
  130 DLTX1 = X1 - (X(K-2)-XOFF)/XFAC
      DLTY1 = Y1 - (Y(K-2)-YOFF)/YFAC
      DLTX2 = X2 - X1
      DLTY2 = Y2 - Y1
      T1 = DLTX1*DLTX1 + DLTY1*DLTY1
      T2 = DLTX2*DLTX2 + DLTY2*DLTY2
      T3 = 2.*SQRT(T1*T2)
      T2 = -T2
      T1 = T3 + T1
      GO TO 150
C
C...Intermediate data point (1<K<NET)
  140 DLTX1 = X2 - X1
      DLTY1 = Y2 - Y1
      DLTX2 = (X(K+1)-X(K))/XFAC
      DLTY2 = (Y(K+1)-Y(K))/YFAC
      T1 = DLTX1*DLTX1 + DLTY1*DLTY1
      T2 = DLTX2*DLTX2 + DLTY2*DLTY2
C
  150 E = DLTX1*T2 + DLTX2*T1
      F = DLTY1*T2 + DLTY2*T1
      G = SQRT(E*E+F*F)
      IF (G.NE.0.)  G = 1./G
      C2 = G*E
      S2 = G*F
      IF (K.EQ.1)  GO TO 180
C
      U = X2 - X1
      V = Y2 - Y1
      G = U*U + V*V
      A = C1*C2 + S1*S2
C
C...Check if (X2,Y2) is more than 'DELTA' from (X1,Y1)
      IF (G.GE.DLTSQ)  GO TO 200
C
C...Distinguish between close points and coincident points
      IF (G.GT.0.)  GO TO 170
C
C...Test for matching tangents
      IF (A.LE.0.99996)  GO TO 180
C
C...(X1,Y1),(X2,Y2) less than 'DELTA' apart, hop to next point.
  170 K = K + 1
      IF (K.LE.NET)  GO TO 110
C
  180 CALL PLOT (X2,Y2,IPEN)
      H = DELT
      IPEN = 2
      GO TO 320
C
C...Cubic coefficients for X and Y
  200 A = 7. - A
      E = C1 + C2
      F = S1 + S2
      B = U*E + V*F
      T = SQRT(B*B+2.*A*G)
      C = (T+B)/G
      T = 3.*(T-B)/A
      G = C/12.
      A = G*(C*U-3.*E)
      B = G*(C*V-3.*F)
      U = G*(C2-C1) + A
      V = G*(S2-S1) + B
      C = -C/9.
      A = A*C
      B = B*C
      G = H
C
C...X AND Y coordinates of arc are given as parametric cubics with
C   parameter going from zero to T and held in G.  The increment is DELTA.
C   G is set initially to space the first point of the new arc
C   at distance DELTA from the last point of the previous arc.
C
C...Generate approximation for each segment
  220 E = G*(G*(A*G+U)+C1) + X1
      F = G*(G*(B*G+V)+S1) + Y1
      CALL PLOT (E,F,IPEN)
      IPEN = MPEN - IPEN
      G = G + DELT
      H = G - T
      IF (H.LE.0.)  GO TO 220
C
C...Arc (X1,Y1) to (X2,Y2) complete, setup for next arc
  320 X1 = X2
      Y1 = Y2
      C1 = C2
      S1 = S2
      K = K + 1
      IF (K.LE.NET)  GO TO 110
C
C
C...Close to last point of curve
      CALL PLOT (X2,Y2,IPEN)
  500 CALL PLOTOF
      RETURN
      END


      SUBROUTINE AXIS (X,Y,LABEL,NCHAR,AXLEN,ANGLE,FVAL,DV)
C...Plots labeled axis with tic marks and annotations
C
C    Input:   X,Y    Starting coordinates for axis (REAL)
C
C             LABEL  Text string for labeling the axis
C
C             NCHAR  Number of characters in the axis label (INTEGER)
C                NCHAR>0 annotations generated above axis
C                NCHAR<0 annotations generated below axis
C
C             AXLEN  Axis length in inches (REAL)
C             ANGLE  Axis angle in degrees (positive CCW) (REAL)
C
C             FVAL   First annotation value (REAL)
C             DV     Delta annotation value (REAL)
C
C        Calls:  NUMBER, SYMBOL
C
C...Note:
C   If your compiler supports passing character data as integers without
C   complaining and you want old style Versatec SYMBOL compatibility (so 
C   you can call SYMBOL with either characters stuffed into an integer 
C   array or with integer arguments to specify plot symbols) you can use
C   the sections of the following code marked with C+++OldVersatec comments. 
C
C---Declaration as a character variable
      CHARACTER*(*) LABEL
C---Declaration of as integer or byte variable
C+++OldVersatec (use either LOGICAL*1, INTEGER*1 or BYTE declaration)
C      LOGICAL*1 LABEL(1)
C      INTEGER*1 LABEL(1)
C      BYTE      LABEL(1)
C+++OldVersatec 
C
      DATA RADN/0.01745329/
C
C
C...Side of axis to annotate and label
      SIDE = +1.
      NC = NCHAR
C...NCHAR<0 means plot below axis
      IF (NC.LT.0) THEN
        NC = -NC
        SIDE = -1.
      ENDIF
C
C...Value of 'DV' exponent
      EXP = 0.0
      ADV = ABS (DV)
C
C...Check for zero delta annotation value?
      IF (ADV.NE.0.) THEN
C
C...Get exponent by dividing by decades
   20  IF (ADV.LT.99.) GO TO 40
       ADV = ADV/10.
       EXP = EXP + 1.
       GO TO 20
C
   30  ADV = ADV*10.
       EXP = EXP - 1.
C...If too small cut the exponent
   40  IF (ADV.LT.0.01) GO TO 30
C
      ENDIF
C
C...Normalized 'FVAL' and 'DV'
      VAL = FVAL*(10.**(-EXP))
      ADV = DV*(10.**(-EXP))
C
C...Angular orientation variables
      T2 = ANGLE*RADN
      SINA = SIN (T2)
      COSA = COS (T2)
C
      DX = -0.1
      DY = 0.15*SIDE - 0.05
      XX = X + DX*COSA - DY*SINA
      YY = Y + DY*COSA + DX*SINA
C
C...Annotate axis
      NTIC = AXLEN + 1.0
      DO I=1,NTIC
        CALL NUMBER (XX,YY,0.105,VAL,ANGLE,2)
        VAL = VAL + ADV
        XX = XX + COSA
        YY = YY + SINA
      END DO
C
C...Label axis
      T2 = NC
C
C...Do we have a valid exponent?
      IF (EXP.NE.0.) T2 = NC + 6
C
      DX = -0.07*T2 + 0.5*AXLEN
      DY = 0.325*SIDE - 0.075
      XX = X + DX*COSA - DY*SINA
      YY = Y + DY*COSA + DX*SINA
      CALL SYMBOL (XX,YY,0.14,LABEL,ANGLE,NC)
C
C...Plot exponent
      IF (EXP.NE.0.) THEN
        CALL SYMBOL (999.,999.,0.14,'  *10',ANGLE,5)
        T2 = NC + 5
        XX = XX + (T2*COSA - 0.8*SINA)*0.14
        YY = YY + (T2*SINA + 0.8*COSA)*0.14
        CALL NUMBER (XX,YY,0.07,EXP,ANGLE,-1)
      ENDIF
C
C...Draw axis and tic marks
      DX = -0.07*SIDE*SINA
      DY = +0.07*SIDE*COSA
      XX = X
      YY = Y
      CALL PLOT (XX,YY,3)
      DO I=1,NTIC
        CALL PLOT (XX,YY,2)
        CALL PLOT (XX+DX,YY+DY,2)
        CALL PLOT (XX,YY,3)
        XX = XX + COSA
        YY = YY + SINA
      END DO
C
      CALL PLOTOF
      RETURN
      END


      SUBROUTINE AXIS2 (X,Y,LABEL,NCHAR,CSCAL,AXLEN,DAX,ANGLE,FVAL,DV)
C...Plots labeled axis with tic marks and annotations
C
C    Input:   X,Y    Starting coordinates for axis (REAL)
C
C             LABEL  Text string for labeling the axis
C
C             NCHAR  Number of characters in the axis label (INTEGER)
C                NCHAR>0 annotations generated above axis
C                NCHAR<0 annotations generated below axis
C
C             CSCAL  Character height scale factor
C             AXLEN  Axis length in inches (REAL)
C             DAX    Distance between annotations in inches
C                DAX>0 first annotation value plotted
C                DAX<0 first annotation value not plotted
C                (using DAX=1.0 corresponds to AXIS routine)
C             ANGLE  Axis angle in degrees (positive CCW) (REAL)
C
C             FVAL   First annotation value (REAL) [not plotted if DAX < 0]
C             DV     Delta annotation value (REAL)
C
C        Calls:  NUMBER, SYMBOL
C
C...Note:
C   If your compiler supports passing character data as integers without
C   complaining and you want old style Versatec SYMBOL compatibility (so 
C   you can call SYMBOL with either characters stuffed into an integer 
C   array or with integer arguments to specify plot symbols) you can use
C   the sections of the following code marked with C+++OldVersatec comments. 
C
C---Declaration as a character variable
      CHARACTER*(*) LABEL
C---Declaration of as integer or byte variable
C+++OldVersatec (use either LOGICAL*1, INTEGER*1 or BYTE declaration)
C      LOGICAL*1 LABEL(1)
C      INTEGER*1 LABEL(1)
C      BYTE      LABEL(1)
C+++OldVersatec 
C
      DATA RADN/0.01745329/
C
      ADAX = ABS(DAX)
      ISTART = 1
      IF(DAX.LT.0.0) ISTART = 2
C
C...Character heights
      CHARH = 0.140*ADAX*CSCAL
      RNUMH = 0.105*ADAX*CSCAL
C
C
C...Side of axis to annotate and label
      SIDE = +1.
      NC = NCHAR
C...NCHAR<0 is lower side
      IF (NC.LT.0) THEN
        NC = -NC
        SIDE = -1.
      ENDIF
C
C...Exponent?
      EXP = 0.0
      ADV = ABS (DV)
C
C...Check for zero delta annotation value?
      IF (ADV.NE.0.) THEN
C
C...Check exponent by dividing by decades
   20  IF (ADV.LT.99.) GO TO 40
       ADV = ADV/10.
       EXP = EXP + 1.
       GO TO 20
C
   30  ADV = ADV*10.
       EXP = EXP - 1.
C...If too small cut the exponent
   40  IF (ADV.LT.0.01) GO TO 30
C
      ENDIF
C
C...Normalize 'FVAL' AND 'DV'
      AFVAL = FVAL*(10.**(-EXP))
      ADV = DV*(10.**(-EXP))
C
C...Angular orientation variables
      T2 = ANGLE*RADN
      SINA = SIN (T2)
      COSA = COS (T2)
C
      DY = 1.4*RNUMH*SIDE - 0.5*RNUMH
C
C...Annotate axis
      NTIC = INT(AXLEN/ADAX) + 1
      DO I=ISTART, NTIC
        RNT = FLOAT(I-1)
        VAL = AFVAL + ADV*RNT
C...Number of digits before decimal point
        NV10 = INT(ABS(VAL)/10.)
        NDIG = 1
        IF(NV10.GT.0) NDIG = NDIG + 1
C...Add one for minus sign
        IF(VAL.LT.0.) NDIG = NDIG + 1
C
        DX = -(FLOAT(NDIG) + 0.5)*0.84*RNUMH
        XX = X + DX*COSA - DY*SINA + ADAX*COSA*RNT
        YY = Y + DY*COSA + DX*SINA + ADAX*SINA*RNT
        CALL NUMBER (XX,YY,RNUMH,VAL,ANGLE,2)
      END DO
C
C...Label axis
      T2 = FLOAT(NC)
C
C...Do we have a valid exponent?
      IF (EXP.NE.0.) T2 = FLOAT(NC + 6)
C
      DX = -0.5*CHARH*T2 + 0.5*AXLEN
      DY = (1.5*RNUMH + 1.5*CHARH)*SIDE - 0.5*CHARH
      XX = X + DX*COSA - DY*SINA
      YY = Y + DY*COSA + DX*SINA
      CALL SYMBOL (XX,YY,CHARH,LABEL,ANGLE,NC)
C
C...Plot exponent
      IF (EXP.NE.0.) THEN
        CALL SYMBOL (999.,999.,CHARH,'  X 10',ANGLE,6)
        T2 = FLOAT(NC + 6)
        XX = XX + (T2*COSA - 0.75*SINA)*CHARH
        YY = YY + (T2*SINA + 0.75*COSA)*CHARH
        CALL NUMBER (XX,YY,0.5*CHARH,EXP,ANGLE,-1)
      ENDIF
C
C...Tic marks
      DX = -0.4*RNUMH*SIDE*SINA
      DY = +0.4*RNUMH*SIDE*COSA
      DO I=1, NTIC
        RNT = FLOAT(I-1)
        XX = X + COSA*ADAX*RNT
        YY = Y + SINA*ADAX*RNT
        CALL PLOT (XX,YY,3)
        CALL PLOT (XX+DX,YY+DY,2)
      END DO
C
C...Axis
      XX = X + COSA*AXLEN
      YY = Y + SINA*AXLEN
      CALL PLOT (X,Y,3)
      CALL PLOT (XX,YY,2)
C
      CALL PLOTOF
      RETURN
      END



      SUBROUTINE SCALE (ARRAY,AXLEN,NPTS,INC)
C...Determines scale factor and offset for elements in array and 
C   installs values in last 2 array elements
C
C    Input:  ARRAY  is an array of data points (REAL)
C
C            AXLEN  Axis length in inches (REAL)
C            NPTS   Number of data points to be scaled (INTEGER)
C            INC    Increment between points in ARRAY (INTEGER)
C
      DIMENSION ARRAY(1)
C
      DIMENSION UNITS(7)
      DATA UNITS(1)/1./,UNITS(2)/2./,UNITS(3)/4./,UNITS(4)/5./
      DATA UNITS(5)/8./,UNITS(6)/10./,UNITS(7)/20./
C
C
C...Min and max values of 'ARRAY' accessed by 'INC' stride
      K = IABS(INC)
      J = NPTS*K
      ARMIN = ARRAY(1)
      ARMAX = ARMIN
      DO I=1,J,K
        AR = ARRAY(I)
        ARMIN = MIN(ARMIN,AR)
        ARMAX = MAX(ARMAX,AR)
      END DO
C
C...Delta value for unit interval
      DV = (ARMAX - ARMIN)/AXLEN
C
C...If negative or zero scale use the sum of min and max
      IF (DV.LE.0.) THEN
        DV = ABS((ARMIN + ARMIN)/AXLEN) + 1.
      ENDIF
C
C...Exponent for DV
      A = 10.0**(IFIX (LOG10 (DV) + 1000.) - 1000)
C
C...Normalized 'DV' value (1<DV<10)
      DV = DV/A - 0.01
C...Find appropriate unit and range for DV value from desirable unit list
      DO I=1,6
        IF (UNITS(I).GE.DV) GO TO 40
      END DO
C
C...Set direction for rounding
 40   SGNF = 0.01
      IF (ARMIN.LT.0.) SGNF = -0.99
C
C...Delta value and min value from normalized unit and exponent
 50   DV = UNITS(I)*A
      TMIN = DV*AINT (ARMIN/DV + SGNF)
C
C...Check to make sure the selected scale is big enough
      IF((TMIN + (AXLEN + 0.01)*DV).GE.ARMAX) GO TO 60
      TMIN = AINT (ARMIN/A + SGNF) *A
      IF((TMIN + (AXLEN + 0.01)*DV).GE.ARMAX) GO TO 60
      I = I + 1
      GO TO 50
C
C
C...Recompute min value
   60 TMIN = TMIN - DV*AINT ((AXLEN + (TMIN - ARMAX)/DV)/2.0)
      IF (ARMIN*TMIN.LE.0.0) TMIN = 0.0
C
C...Reverse direction if necessary
      IF (INC.LE.0) THEN
        TMIN = TMIN + DV*AINT (AXLEN + 0.5)
        DV = -DV
      ENDIF
C
C...Install scale and offset into array
      J = J + 1
      ARRAY(J) = TMIN
      K = J + K
      ARRAY(K) = DV
C
      RETURN
      END




      SUBROUTINE SYMBL (X,Y,HGT,ISYM,ANGLE,NC)
C...Plot a symbol at the specified location with specified size and angle 
C
C      This routine accepts an integer symbol selector
C      instead of a character string as its argument
C
C             X,Y     starting coordinate for the symbol
C             HGT     symbol height specification (in inches)
C             ISYM    integer index corresponding to the symbol
C             ANGLE   angle at which the symbol is plotted
C             NC      NC must be NC<0
C                 NC=-1  move to x,y with 'pen' up;   plot symbol #ISYM
C                 NC<-1  move to x,y with 'pen' down; plot symbol #ISYM
CC...Note: 
C   You might not need the SYMBL routine, if your compiler supports passing
C   character literals as integers without complaining and you enable old 
C   style Versatec SYMBOL calls with integer arguments.  See SYMBOL below...
C
        IF (NC.EQ.0) RETURN
        IF (NC.GT.0) THEN
          WRITE(*,*) 'Bad NC argument to SYMBL ',NC
          RETURN
        ENDIF
      CALL SYMBOL (X,Y,HGT,char(ISYM),ANGLE,NC)
      RETURN
      END


      SUBROUTINE SYMBOL(XZ,YZ,HGT,ITEXT,ANGLE,NZ)
C...Plots characters using stroke representation of char. set
C
C        XZ,YZ  starting coordinate for the text
C        HGT    character height (in inches)
C        ITEXT   alphanumeric text to be generated
C        ANGLE  angle at which the character line is plotted
C        NC     number of characters to be plotted
C          NC>0   alpha text, number of characters to be plotted
C          NC=0   plot single character,right-justified in text
C          NC=-1  move to x,y with 'pen' up;   plot symbol #TEXT
C          NC<-1  move to x,y with 'pen' down; plot symbol #TEXT
C
C...Note:
C   If your compiler supports passing character data as integers without
C   complaining and you want old style Versatec SYMBOL compatibility (so 
C   you can call SYMBOL with either characters stuffed into an integer 
C   array or with integer arguments to specify plot symbols) you can use
C   the sections of the following code marked with C+++OldVersatec comments. 
C   In that case you don't need the SYMBL routine, just call SYMBOL with 
C   integer arguments.  However, with integer arguments, you lose some of
C   the nice things about character string manipulation.
C
C---Declaration of ITEXT as a character variable
      CHARACTER*(*) ITEXT
C
C---Declaration of ITEXT as integer or byte variable
C+++OldVersatec (use either LOGICAL*1, INTEGER*1 or BYTE declaration)
C      LOGICAL*1 ITEXT(1)
C      INTEGER*1 ITEXT(1)
C      BYTE      ITEXT(1)
C+++OldVersatec 
C
      DIMENSION XA(14),YA(14)
      DIMENSION ASIN(5),ACOS(5)
      INTEGER*4 IRAM, AND
      INTEGER*4 NCHR,KVAL,NODES,
     &          MSKALL,MSK4,MSK5,MSK7,MSK8,MSK11,
     &          KHAR(128),NODE(468), 
     &          NOD1(160), NOD2(160), NOD3(160) 
C
C...Assemble the NODE array (this silliness necessary because of f77 
C   limitation on number of continuation lines) 
      EQUIVALENCE (NODE(1)  ,NOD1(1))
      EQUIVALENCE (NODE(161),NOD2(1))
      EQUIVALENCE (NODE(321),NOD3(1))
C...Constants
      DATA NBITS /16/, NBYTES /2/
      DATA NCHRS/128/
      DATA RADCO/0.01745329/
      DATA EPSIL/0.0000277 /
      DATA ASIN /0.,1.,0.,-1.,0./,
     &     ACOS /1.,0.,-1.,0.,1./
      DATA MSKALL/-1/, MSK4 /15/, MSK5 /31/, 
     &     MSK7 /127/, MSK8 /255/, MSK11 /2047/
C...Static variables (changed within SYMBOL)
      DATA FCTR/0.7/, FACC/0.0/
      DATA THETA/0.0/
      DATA  ANCC, ANCS /1.0, 0.0/
      DATA XC, YC /0.0, 0.0/,
     &     XT, YT /0.0, 0.0/,
     &     XO, YO /0.0, 0.0/
      DATA XA / 0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0./,
     &     YA / 0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0./
C
C...Characters encoding stroke commands for drawing
      DATA KHAR/     7,   267,   645,   839,  1095,  1350,  1574,  1797,
     1            1991,  2247,  2513,  3086,  3557,  3747,  3877,  4065,
     2            4129,  4192,  4232,  4519,  4772,  4928,  4967,  5223,
     3            5480,  5760,  5793,  5863,  6117,  6306,  6405,  6604,
     4            7008,  7046,  7272,  7562,  7915,  8301,  8745,  9059,
     5            9187,  9315,  9447,  9700,  9861, 10049, 10116, 10273,
     6           10344, 10629, 10823, 11084, 11498, 11848, 12138, 12484,
     7           12655, 13163, 13546, 13899, 14274, 14372, 14530, 14636,
     8           15055, 15561, 15883, 16263, 16518, 16743, 16998, 17225,
     9           17543, 17799, 18052, 18215, 18466, 18564, 18723, 18859,
     A           19238, 19467, 19849, 20171, 20548, 20709, 20898, 20996,
     1           21156, 21317, 21510, 21731, 21857, 21923, 22050, 22145,
     2           22224, 22754, 22856, 23143, 23403, 23787, 24169, 24486,
     3           24712, 24998, 25229, 25675, 26056, 26343, 26592, 26624,
     4           26660, 26829, 27269, 27461, 27651, 27786, 28129, 28193,
     5           28263, 28516, 28679, 28938, 29284, 29444, 29604, 29764/
C
C...Node data for stroked characters (in 3 separate arrays due to problem
C   with some f77 compilers for continuation lines...).  You could assemble
C   these all into one array and dispense with the equivalence statements if
C   your compiler supported lots of continuation lines.
      DATA NOD1/  8740,  1024, 16452,  9250,  8740,  5123,   272, 12353,
     1           17204,  9250,  8740,   321,  9250,  8770, -4060,  8432,
     2             546,  8772, -4092, 16624,    34,  8740,   544, 16932,
     3            8738,  8228,   578,  9250,  8704, 17412, 16418,  8772,
     4            1092,    64,    34,  8772, -4092,  8944,  8226,  8772,
     5           -4045, 12608, -4096,  4371,  1264,  4401, -4077, 13090,
     6            8770, -4060,  8432,   546, 17648,  1088, -4096,  8738,
     7           17412, 16384,  8738,  9248,  8754, 22642,  9862, 12834,
     8           25122,  9972,  8497, 16965, 22087, 19003, 11047, 26608,
     9           25893, -4061, 25365, 30038, 21621, -3292, 25840, 26150,
     A           -4045, 22307, 25584, 17480, -4058, 26209, 20802, 17718,
     1           18250, 23403, -2790,-30190,  8497, 16969, 23146, 31012,
     2           21605, 26199, 10022, 16998,  5157, 13651, 25460,  5157,
     3           13651, 25460, -3978, 25941, 14119,  5872, 18756, -4046,
     4           17234, 12887, 18777, 22512, 14121, 14647,  9316, 21587,
     5           22358, 26150, 13879, 13129, 17136,  9043, 25701, 22070,
     6           10040, 26680, 10281, 14648, -3991,  8944, 21347, 25170,
     7           21346, 14136, 18776,  9508, 13123, 25687, 18777, 22377,
     8           22611, 25129, 14387,  8741, 26096,  9063, -4057, 25412,
     9           18672,  9830, 16946, 13123, 16945,  9573, 16946, 13123/
C
      DATA NOD2/ 16930, 26984, 22841, 10275, 12882, 25448, 14409, 17136,
     1           12882, 10297, 22888, 26148,  8802, 10297, 22888, 26454,
     2           13910, 25955, 21042,  8996, 25840, 22866, -4055,  9456,
     3           16994,  9010, 21091, 25942,  9769, 26934, 22117, 25426,
     4           12835, 10297, 22888, 10601, 26691, 16982, 26472, 22841,
     5           10279, 13910, 25955, 21042,  8997, 13859, 12882, 25448,
     6           22841, 10278, 13653, 26165, 13894, 17717, -4030, 12851,
     7           17218, 13622, 17989, 13808, 16946, 13123, 16945, 26662,
     8           25636, 25840, 26150,  9318, 10290, 17234, 13040, 17478,
     9           22119, 26713, 14632, 26199, 18230, 13636, 21605, 26456,
     C           14375,  9267, 21348,  8741, 26096,  9512, 14681, 26722,
     1           25445, 22054, 22119, 26713, 10530, 21091, 26713, 14632,
     2            9010, 21091, 26713, 10530, 21091, 26729, 10534, 22256,
     3            9762, 25193, 10534, 22256,  9762, 26713, 14632,  9010,
     4           21091, 25941,  8745, -4058, 26352, 26978, 12882, -4030,
     5           18928, 14681,  9010, 21091, 26921,  8944,  9577, -4025,
     6           25129,  8802,  8745, 17769, 25193, 25129,  8775, 27120,
     7           26713, 14632,  9010, 21091, 26658, 10585, 26727, 22054,
     8           26713, 14632,  9010, 21091, 26864, 17506,  8745, 22888,
     9           26454,  9814, 25954, 26713, 14632, 10038, 22117, 25426/
C
      DATA NOD3/ 12835, 10601, -4023, 16937,  9010, 21091, 26921, 17001,
     1           10530, 18018, 26921, 25328,  8809, 10567, 17136, 18281,
     2           10601,  8802, -4042, 22121, 10530, 25129, 25122, 25193,
     3           10530, 18018,  4481, 26455, 17975, 10006,  5412, 13381,
     4           17989, 21604, 30070, 26405, 30068,  8759, 13379, 21348,
     5           26468, 29491, 14320, 10087, -4009, 21314, 18672, 14118,
     6            9267, 21348, 26199, 14135,  9764, 13139, 25702, 22327,
     7           -4075, 29990, 13876, 17236, 22118, -4030, 18467, 26352,
     8            9782, 21347, 10020, 13124, 18244, 21348, 26408, 14149,
     9           25584,  8773, 26455, 17975, 10006,  5412, 13381, 17989,
     E           21604, 22600, 14150, 22117, 25683, 17204, 13638, 26439,
     1           13876, 17251, -3995,  9766, 14150, 13382, 22374, 21233,
     2           -3471,  8534, 10874,  5477, -4045, 13380, 17203, -4042,
     3           14151, 17974, 26662, 25840, 25379, 25379, -4060, 26152,
     4            4679, 29202, 26199, 14118,  9267, 21348, -4030, 18453,
     5           29985, 10792, 14664, 17136, 18521, 26659, 13378, 19066,
     6           16969, -4044, 21744, 14424, 16969, -4044, 21744, 14424,
     7           -4042, 22037, 13876,  5493,  9063, -4057, 25410, 18743,
     8           22345, 16948, 21570, 18688,     0,     0,     0,     0,
     9               0,     0,     0,     0,     0,     0,     0,     0/
C
      X = XZ
      Y = YZ
      NC = NZ
C
C...Save and set line pattern, Symbols/text only drawn with solid lines
C...Process a draw to character position before switching line pattern 
      CALL GETPAT(IMASK)
      IF (NC.LT.-1)  CALL PLOT(X,Y,2)
      IF(IMASK.NE.-1) THEN
        MSK = MSKALL
        CALL NEWPAT(MSK)
      ENDIF
C
C
C...Get set to loop through the text array character by character
      K = 0
      IC  = 3
      DIV = 7.0
C
C
C...Extract next character from text array, masking off high bit
  180 K = K + 1
C
C---For declaration of ITEXT as a character variable
      NCHR = ICHAR(ITEXT(K:K))
C---Declaration of ITEXT as integer or byte variable
C+++OldVersatec (use either LOGICAL*1, INTEGER*1 or BYTE declaration)
C      NCHR = ITEXT(K)
C+++OldVersatec 
C
C
      NCHR = IRAM(NCHR,0,MSK7)
C
C...NC<0 Centered symbol
C   NC=0 Right-justified symbol
C   NC>0 Regular left-justified text
C
      IF(NC.LT.0) THEN
        IF (NCHR.LE.13) DIV = 4.0
      ENDIF
      NCC = NCHR
C
C...On first character, set character height, angle and position
C
      IF(K.EQ.1) THEN
C...Use current height and angle?
        IF (HGT.GT.0.0) THEN
         ISTAT = 1
         FCT = HGT/DIV
C...Calculate a new theta if necessary
C   use stored quadrant angles if at n(90 deg).
        IF (ANGLE.NE.THETA) THEN
          FACC = FCT
          THETA = ANGLE
          ANG = MOD(ANGLE,360.0)
          IF (ANG.LT.0) ANG = 360.0 - ANG
          I = (ANG + EPSIL)/90.0
          A = I*90.0
          IF (ABS(ANG-A).GT.EPSIL) THEN
            ANCC = THETA*RADCO
            ANCS = SIN(ANCC)
            ANCC = COS(ANCC)
           ELSE
            ANCS = ASIN(I+1)
            ANCC = ACOS(I+1)
          ENDIF
          CALL SOFFSET(FACC,ANCC,ANCS,XA,YA)
         ENDIF
C...Calculate offsets for new FACC and/or ANGLE
         IF (FCT.NE.FACC) THEN
           FACC = FCT
           CALL SOFFSET(FACC,ANCC,ANCS,XA,YA)
         ENDIF
        ENDIF
C...Set character position
C...If X,Y coordinate = 999.0; use previous value(s) of X and/or Y
        IF (X.NE.999.0) THEN
          XO = X-XA(3)+YA(3)
          XC = XO
        ENDIF
        IF (Y.NE.999.0) THEN
          YO = Y-XA(3)-YA(3)
          YC = YO
        ENDIF
        X = XC
        Y = YC
      ENDIF
C
C
C...Extract node count (5 bits) and index (11 bits) into node array
      KDEX = MOD(NCC,NCHRS) + 1
      KVAL = KHAR(KDEX)
      NDKNT = AND(KVAL,MSK5)
      INDX = IRAM(KVAL,5,MSK11)
C
C...Compute word index into node array for first node
      NWD = INDX/NBYTES + 1
C
C...Compute byte index(NBT = shift count)into node word for node start
      NBT = -8*MOD(INDX,NBYTES) - 8
C
C...Node processing loop
  210 IF(NBT+NBITS.LT.0) THEN
        NWD = NWD + 1
        NBT = -8
      ENDIF
C
C...Extract the next node.
      NODES = IRAM(NODE(NWD),NBT,MSK8)
      NBT = NBT - 8
      NODEY = AND(NODES,MSK4)
      NODEX = IRAM(NODES,4,MSK4)
C
C...Check for special control functions (NODEX = 15)
      IF (NODEX.GE.15) THEN
C - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
C...Decode special function (y-offset)
C...Blank,superscript,subscript,carriage return,back space,null char?
C
C...Blank character
      IF(NODEY.EQ.0) THEN
        IC = 3
        GO TO 245
C...Superscript set/reset code: (first test existing state)
       ELSEIF(NODEY.EQ.1) THEN
        IF(ISTAT.EQ.0) THEN
C - For ISTAT=0 return to normal STATE=1
         ISTAT = 1
         FACC = FACC/FCTR
         CALL SOFFSET(FACC,ANCC,ANCS,XA,YA)
         X = X-YA(2)
         Y = Y+XA(2)
        ELSEIF(ISTAT.EQ.1) THEN
C - For ISTAT=1 Set superscript mode,ISTAT=2
         ISTAT = 2
         X = X-YA(5)
         Y = Y+XA(5)
         FACC = FACC*FCTR
         CALL SOFFSET(FACC,ANCC,ANCS,XA,YA)
        ENDIF
C - For ISTAT=2 do nothing and branch to next character
C
C
C...Subscript set/reset code: (first test existing state)
       ELSEIF(NODEY.EQ.2) THEN
C - For ISTAT=0  branch to next character
C - For ISTAT=1) set subscript mode, ISTAT=0
        IF(ISTAT.EQ.1) THEN
         ISTAT = 0
         X = X+YA(2)
         Y = Y-XA(2)
         FACC = FACC*FCTR
         CALL SOFFSET(FACC,ANCC,ANCS,XA,YA)
C - For ISTAT=2 return to normal mode, ISTAT=1
        ELSEIF(ISTAT.EQ.2) THEN
         ISTAT = 1
         FACC = FACC/FCTR
         CALL SOFFSET(FACC,ANCC,ANCS,XA,YA)
         X = X+YA(5)
         Y = Y-XA(5)
        ENDIF
C...Carriage return
       ELSEIF(NODEY.EQ.3) THEN
        X = XO + YA(13)
        Y = YO - XA(13)
        XO = X
        YO = Y
C...Backspace
       ELSEIF(NODEY.EQ.4) THEN
        X = X - XA(8)
        Y = Y - YA(8)
C...Null character
       ELSEIF(NODEY.EQ.5) THEN
      ENDIF
C
C...Go on to next character
      GO TO 260
      ENDIF
C - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
C...Process move to node
      NODEX = NODEX + 1
      NODEY = NODEY + 1
      YT = Y + YA(NODEX) + XA(NODEY)
      XT = X + XA(NODEX) - YA(NODEY)
C
C
C...Plot the character stroke
      CALL PLOT(XT,YT,IC)
      IC = 2
      X = XC
      Y = YC
C
C...Decrement and test node count
  245 NDKNT = NDKNT - 1
C
C...Any nodes yet to be processed?
      IF (NDKNT.GE.0) GO TO 210
      X = X + XA(8)
      Y = Y + YA(8)
C
C...Decrement and test symbol count
  260 XC = X
      YC = Y
      IC = 3
      NC = NC - 1
C
C...Any symbols yet to be plotted?
      IF (NC.GT.0) GO TO 180
C
C...End of SYMBOL processing, if necessary restore line pattern
      IF(IMASK.NE.-1) THEN
        CALL NEWPAT(IMASK)
      ENDIF
      RETURN
      END

      SUBROUTINE SOFFSET(FACC,ANCC,ANCS,XA,YA)
C...Calculates character offsets using current factor and direction 
C   sine and cosine
      DIMENSION XA(14), YA(14)
C
      Z = FACC*ANCC
      W = FACC*ANCS
      XI = Z
      YI = W
      DO L=2,14
        XA(L) = Z
        YA(L) = W
        Z = Z+XI
        W = W+YI
      END DO
      RETURN
      END


      SUBROUTINE NUMBER (X,Y,HEIGHT,FPN,ANGLE,NDIG)
C...Plot number as a string of characters
C
C     (X,Y) = starting coordinates for 1st char (real)
C    HEIGHT = character height (real)
C       FPN = number to be converted to digits and plotted (real)
C     ANGLE = angle at which numeric string is to be plotted
C             in degrees measured from the x-axis (real)
C      NDIG = specification of the number of digits and the type
C             of numeric string to be plotted (integer)
C             > 0 = number of digits to the right of the decimal
C                   point to be plotted (last digit is rounded)
C             = 0 = rounded integer portion of fpn is plotted
C                   with a decimal point
C            = -1 = rounded integer portion of fpn is plotted
C                   without the decimal point
C            < -1 = rounded integer portion of fpn is plotted
C                   after having the least significant digits
C                   truncated (IABS(NDEC)-1 digits are truncated)
C
C        CALLS:  SYMBOL
C
C...Note:
C   If your compiler supports passing character data as integers without
C   complaining and you want old style Versatec SYMBOL compatibility (so 
C   you can call SYMBOL with either characters stuffed into an integer 
C   array or with integer arguments to specify plot symbols) you can use
C   the sections of the following code marked with C+++OldVersatec comments. 
C
C---Declaration using character variables
      CHARACTER*1 MINUS, IPOINT, CHDIG
C---Declaration of ITEXT as integer or byte variable
C+++OldVersatec (use either LOGICAL*1, INTEGER*1 or BYTE declaration)
C      LOGICAL*1 MINUS, IPOINT, CHDIG
C      INTEGER*1 MINUS, IPOINT, CHDIG
C      BYTE      MINUS, IPOINT, CHDIG
C+++OldVersatec 
C
      DATA MINUS/'-'/,IPOINT/'.'/
C
      IZERO = ichar('0')
C
      T1 = FPN
      XZ = X
      YZ = Y
C...Number negative? 
      IF (T1.LT.0) THEN
        CALL SYMBOL (XZ,YZ,HEIGHT,MINUS,ANGLE,1)
        XZ = 999.
        YZ = 999.
        T1 = -T1
      ENDIF
C
C...Set working digit count
      ND = -NDIG
C
C...Integer only to be plotted? 
      IF (NDIG.LE.0) THEN
C...Round and truncate for integer
        IF(NDIG.EQ.0)  ND = 1
        ND = ND - 1
        T2 = FLOAT (IFIX((T1 + 0.5)/(10.**ND))) + 0.5
        ND = 0
        IF (NDIG.EQ.0) ND = -1
       ELSE
C...Round for fraction
        T2 = T1 + 0.5/(10.**NDIG)
      ENDIF
C
C...Find number of digits to the left of decimal point
      NL = 1
C...Any more digits to the left of the d.p.
   60 IF (T2.GE.10.) THEN
        T2 = T2/10.
        NL = NL + 1
        GO TO 60
      ENDIF
C
C...Set plottable digit count
      NP = NL - ND
C...Bad digit count?
      IF (NP.LE.0) NP = 1
C
C...Plot decimal point (NL=0)
   80 IF(NL.EQ.0) THEN
C...No decimal point?
        IF (NDIG.LT.0) GO TO 120
        CALL SYMBOL (XZ,YZ,HEIGHT,IPOINT,ANGLE,1)
        IF (NDIG.NE.0) NP = NP + 1
       ELSE
C...Plot digit
        IDIG = IFIX(T2)
        T2 = (T2 - FLOAT (IDIG))*10.
C---For declaration of CHDIG as character variable
        CHDIG = char(IDIG + IZERO)
C+++OldVersatec (use either LOGICAL*1, INTEGER*1 or BYTE declaration)
C        CHDIG = IDIG + IZERO
C+++OldVersatec 
        CALL SYMBOL (XZ,YZ,HEIGHT,CHDIG,ANGLE,-1)
      ENDIF
      XZ = 999.
      YZ = 999.
C
C...Count digit
      NP = NP - 1
  120 NL = NL - 1
C
C...More digits to plot?
      IF(NP.GT.0) GO TO 80
      RETURN
      END


      INTEGER*4 FUNCTION IRAM(IWORD,K,MASK)
C...Rotate And Mask 16 bits of a word and mask
C   Note that all inputs and outputs are I*4 (32bit)
C   New 32 bit version of IRAM   HHY 4/1/96
      INTEGER*4 MASK,IWORD,I4,I3,IT1,IT2
      INTEGER   RSHIFT, LSHIFT, AND
      I4=LSHIFT(IWORD,16)
      I3=LSHIFT(MASK,16)
C
      IF(K .NE. 0) THEN
C
        if (K .LT. 0) then
         IT1=LSHIFT(I4,-K)
        else         
         IT1=RSHIFT(I4,K)
        endif
C
        IF(K .GT. 0) then
         IT2=LSHIFT(I4,16-K)
        else
         IT2=RSHIFT(I4,abs(-K-16))
        endif
C
        I4=IT1+IT2
      ENDIF
C
      I4=AND(I4,I3)
      IRAM=RSHIFT(I4,16)
C
      RETURN
      END





