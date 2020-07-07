C***********************************************************************
C    Module:  pltops.f
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

      SUBROUTINE PLTINI(IDEV1)
      INCLUDE 'AVLPLT.INC'

C---- terminate old plot if any
      IF(LPLOT) CALL PLEND
C
C---- initialize new plot
      CALL PLOPEN(SCRNFRAC,IPSLU,IDEV1)
      LPLOT = .TRUE.

      CALL DRAWTOBUFFER

C---- set X-window size in inches (might have been resized by user)
      CALL GETWINSIZE(XWIND,YWIND)
C
      IF(LCREV) THEN
       CALL BGFILL
      ENDIF
C
C---- draw plot page outline offset by margins
      CALL NEWPEN(5)
      IF(XMARG .GT. 0.0) THEN
        CALL PLOTABS(      XMARG,      YMARG,3)
        CALL PLOTABS(      XMARG,YPAGE-YMARG,2)
        CALL PLOTABS(XPAGE-XMARG,      YMARG,3)
        CALL PLOTABS(XPAGE-XMARG,YPAGE-YMARG,2)
      ENDIF
      IF(YMARG .GT. 0.0) THEN
        CALL PLOTABS(      XMARG,      YMARG,3)
        CALL PLOTABS(XPAGE-XMARG,      YMARG,2)
        CALL PLOTABS(      XMARG,YPAGE-YMARG,3)
        CALL PLOTABS(XPAGE-XMARG,YPAGE-YMARG,2)
      ENDIF
      CALL NEWPEN(1)
C
      CALL PLOTABS(XMARG,YMARG,-3)
      CALL NEWCLIPABS( XMARG, XPAGE-XMARG, YMARG, YPAGE-YMARG )

      RETURN
      END ! PLTINI



      SUBROUTINE PLTSEG(VEC,ALF,N)
C...PURPOSE:    Plot portions of vectors in VEC given by normalized
C               segment start and end points in ALF
C
      INCLUDE 'AVLPLT.INC'
      REAL VEC(3,2),ALF(2,N)
C
      XMOD(XTMP) = SF * (XTMP - XOFF)
      YMOD(YTMP) = SF * (YTMP - YOFF)
C
      X1 = VEC(1,1)
      Y1 = VEC(2,1)
C
      DX = VEC(1,2) - VEC(1,1)
      DY = VEC(2,2) - VEC(2,1)
C
      I = 1
      XA = X1 + ALF(1,I)*DX
      YA = Y1 + ALF(1,I)*DY
      CALL PLOT(XMOD(XA),YMOD(YA),3)
C
      XA = X1 + ALF(2,I)*DX
      YA = Y1 + ALF(2,I)*DY
      CALL PLOT(XMOD(XA),YMOD(YA),2)
C
      DO 10 I=2, N
C
        IF(ABS(ALF(1,I)-ALF(2,I-1)) .GT. 1.0E-3) THEN
         XA = X1 + ALF(1,I)*DX
         YA = Y1 + ALF(1,I)*DY
         CALL PLOT(XMOD(XA),YMOD(YA),3)
        ENDIF
C
        XA = X1 + ALF(2,I)*DX
        YA = Y1 + ALF(2,I)*DY
        CALL PLOT(XMOD(XA),YMOD(YA),2)
C
   10 CONTINUE
C
      RETURN
      END ! PLTSEG



      SUBROUTINE PLTPOLY(PTS,NPTS)
C...PURPOSE:   Plot polygon given by NPTS vertices in PTS array of XYZ points
C
      INCLUDE 'AVLPLT.INC'
      REAL PTS(3,*)
C
      XMOD(XTMP) = SF * (XTMP - XOFF)
      YMOD(YTMP) = SF * (YTMP - YOFF)
C
      CALL PLOT(XMOD(PTS(1,1)),YMOD(PTS(2,1)),3)
      DO K=2, NPTS
        CALL PLOT(XMOD(PTS(1,K)),YMOD(PTS(2,K)),2)
      END DO
      CALL PLOT(XMOD(PTS(1,1)),YMOD(PTS(2,1)),2)
      RETURN
      END ! PLTPOLY


      SUBROUTINE PLTINT(PT,N,CHSIZE,LOFFSET)
C...Purpose:   Plot integer N at location PT 
C              with character size CHSIZE
C
      INCLUDE 'AVLPLT.INC'
      REAL PT(3)
      LOGICAL LOFFSET
C
      XMOD(XTMP) = SF * (XTMP - XOFF)
      YMOD(YTMP) = SF * (YTMP - YOFF)
C
      FLTN = FLOAT(N)
      X = XMOD(PT(1))
      Y = YMOD(PT(2))
      IF(LOFFSET) THEN
       X = X + 0.80*CHSIZE
       Y = Y - 0.50*CHSIZE
      ELSE
       NCHAR = 1 + INT(LOG10(ABS(FLTN))+0.01)
       X = X - 0.5*CHSIZE*FLOAT(NCHAR)
       Y = Y - 0.5*CHSIZE
       IF(FLTN.LT.0.0) X = X - CHSIZE
      ENDIF
      CALL PLNUMB(X,Y,CHSIZE,FLTN,0.,-1)
C
      RETURN
      END ! PLTINT


      SUBROUTINE PLTFLT(PT,R,CHSIZE,LOFFSET,NDIG)
C...Purpose:   Plot float R at location PT 
C              with character size CHSIZE
C
      INCLUDE 'AVLPLT.INC'
      REAL PT(3)
      LOGICAL LOFFSET
C
      XMOD(XTMP) = SF * (XTMP - XOFF)
      YMOD(YTMP) = SF * (YTMP - YOFF)
C
      ABSR = ABS(R)
C
C---- determine # of digits to use for diplay
      IF(NDIG.LE.-2) THEN
       ND = 1 - MAX( 0 , INT(LOG10(ABSR)) )
       IF(ABSR*10**ND - AINT(ABSR*10**ND+0.01) .GT. 0.01) ND = ND + 1
       IF(ABSR*10**ND - AINT(ABSR*10**ND+0.01) .GT. 0.01) ND = ND + 1
      ELSE
       ND = NDIG
      ENDIF
C
      X = XMOD(PT(1))
      Y = YMOD(PT(2))
      IF(LOFFSET) THEN
       X = X + 0.85*CHSIZE
       Y = Y - 0.50*CHSIZE
      ELSE
       NCHAR = 1 + INT(LOG10(ABSR)) + 1 + ND
       X = X - 0.5*CHSIZE*FLOAT(NCHAR)
       Y = Y - 0.5*CHSIZE
       IF(R.LT.0.0) X = X - CHSIZE
      ENDIF
      CALL PLNUMB(X,Y,CHSIZE,R,0.0,ND)
C
      RETURN
      END ! PLTFLT


      SUBROUTINE PLTARROW(PT1,PT2)
C...PURPOSE:   Plot an arrow vector from PT1 to PT2
C
      INCLUDE 'AVLPLT.INC'
      REAL PT1(3), PT2(3)
C
      XMOD(XTMP) = SF * (XTMP - XOFF)
      YMOD(YTMP) = SF * (YTMP - YOFF)
C
      XAVE = PT1(1)
      YAVE = PT1(2)
      XHED = PT2(1)
      YHED = PT2(2)
C
      DX = XHED - XAVE
      DY = YHED - YAVE
C
      CALL PLOT(XMOD(XAVE),YMOD(YAVE),3)
      CALL PLOT(XMOD(XHED),YMOD(YHED),2)
C
      X1 = XAVE + 0.8*DX + 0.02*DY
      Y1 = YAVE + 0.8*DY - 0.02*DX
      X2 = XAVE + 0.8*DX - 0.02*DY
      Y2 = YAVE + 0.8*DY + 0.02*DX
      CALL PLOT(XMOD(X1  ),YMOD(Y1  ),2)
      CALL PLOT(XMOD(X2  ),YMOD(Y2  ),2)
      CALL PLOT(XMOD(XHED),YMOD(YHED),2)
C
      RETURN
      END ! PLTARROW




      SUBROUTINE ROTATEPT(PT0,PT1,DIR,COSR,SINR)
C...PURPOSE:  Rotate point PT1 about vector in direction DIR through
C             point PT0 by angle with cosine COSR and sine SINR.
C             Rotated point is returned in PT1.
C
C
      REAL PT0(3), PT1(3), DIR(3), DPT(3), EP(3), EQ(3)
C
      DO L = 1, 3
        DPT(L) = PT1(L) - PT0(L)
      END DO
C-----EP = normal-vector component perpendicular to hinge line
      ENDOT = DOT(DPT,DIR)
      EP(1) = DPT(1) - ENDOT*DIR(1)
      EP(2) = DPT(2) - ENDOT*DIR(2)
      EP(3) = DPT(3) - ENDOT*DIR(3)
C-----EQ = unit vector perpendicular to both EP and DIR
      CALL CROSS(DIR,EP,EQ)
C-----rotated vector consists of sin,cos parts from EP and EQ,
C-    with hinge-parallel component ENDOT restored
      DO L = 1, 3
        DPT(L) = EP(L)*COSR + EQ(L)*SINR + ENDOT*DIR(L)
        PT1(L) = PT0(L) + DPT(L)
      END DO
C
      RETURN
      END ! ROTATEPT


      SUBROUTINE OFFINI       
C---- set initial scaling and offset parameters   
C
      INCLUDE 'AVLPLT.INC'
C
      XRANGE = XMAX-XMIN
      YRANGE = YMAX-YMIN
      IF(XRANGE.EQ.0.) XRANGE = 1.0
      IF(YRANGE.EQ.0.) YRANGE = 1.0
      SF = 0.95*MIN( 1.0/XRANGE , PLOTAR/YRANGE )
      XOFF = XMIN - 0.5*(1.0    - SF*XRANGE)/SF
      YOFF = YMIN - 0.5*(PLOTAR - SF*YRANGE)/SF
C
      RETURN        
      END ! OFFINI  


      SUBROUTINE OFFGET       
C---- Get offsets for zoom from user interaction (mouse)
C
      INCLUDE 'AVLPLT.INC'
      CHARACTER*1 CKEY
C
      SH = 2.0
C
      WRITE(*,*)
      WRITE(*,*) 'Mark off corners of blowup area'
      WRITE(*,*) '(2 spaces default to current area)'       
      CALL GETCURSORXY(XX1,YY1,CKEY)
      CALL PLSYMB(XX1,YY1,SH,3,0.0,0)
      CALL GETCURSORXY(XX2,YY2,CKEY)
      CALL PLSYMB(XX2,YY2,SH,3,0.0,0)
      IF(ABS(XX1-XX2).LT.0.01 .AND. ABS(YY1-YY2).LT.0.01) RETURN      
C
      XOFF = MIN(XX1,XX2)/SF + XOFF
      YOFF = MIN(YY1,YY2)/SF + YOFF
      XDIF = ABS(XX2 - XX1)/SF
      YDIF = ABS(YY2 - YY1)/SF   
      IF(XDIF.EQ.0.0) XDIF = 1.0E-5
      IF(YDIF.EQ.0.0) YDIF = 1.0E-5
      SF = MIN( 1.0/XDIF , PLOTAR/YDIF )     
C
C
C---- Re-center the blowup
C      XDIF = MAX(XDIF,YDIF/0.75)
      YDIF = MAX(0.75*XDIF,YDIF)
      XOFF = XOFF - 0.5*(1.0    - SF*XDIF)/SF
      YOFF = YOFF - 0.5*(PLOTAR - SF*YDIF)/SF
C
      RETURN        
      END ! OFFGET  


      SUBROUTINE BGFILL
      INCLUDE 'AVLPLT.INC'
      REAL XBOX(5), YBOX(5)
C
      XBOX(1) = 0.0
      XBOX(2) = XWIND
      XBOX(3) = XWIND
      XBOX(4) = 0.0
      XBOX(5) = XBOX(1)

      YBOX(1) = 0.0
      YBOX(2) = 0.0
      YBOX(3) = YWIND
      YBOX(4) = YWIND
      YBOX(5) = XBOX(1)

      CALL NEWCOLORNAME('BLACK')
      IF(SCRNFRAC .GT. 0.0) THEN
       CALL POLYLINE(XBOX,YBOX,5,1)
      ELSE
       CALL POLYLINE(YBOX,XBOX,5,1)
      ENDIF
      CALL NEWCOLORNAME('WHITE')
C
      RETURN
      END ! BGFILL
