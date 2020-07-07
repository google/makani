C
C   PPLOT array limits
C
C     NAX    number of points in one polar
C     NPX    number of polars
C     NFX    number of points in one reference polar
C     NDX    number of reference polars
C     ISX    number of airfoil sides
C
C
      INCLUDE 'PINDEX.INC'
C
      PARAMETER (NAX=500, NPX=20, NFX=128, NDX=20, ISX=10)
      CHARACTER*80 FNPOL, FNREF, FNAME
      CHARACTER*32 NAME, LABREF, TITLE
      CHARACTER*16 CODE, CCLEN
      REAL MACH
C
      LOGICAL 
     &  LGRID,LPLOT,
     &  LCDW,LLIST,LEGND,LCLIP,
     &  LCLEN,LAECEN,
     &  LCDH,LCMDOT,LAUTO
C
      COMMON/PPCOM_L/
     &  LGRID,LPLOT,
     &  LCDW,LLIST,LEGND,LCLIP,
     &  LCLEN,LAECEN,
     &  LCDH,LCMDOT,LAUTO
C
      COMMON/PPCOM_I/
     &  IDEV, IDEVRP, IPSLU, ICOL0, NCOLOR,
     &  NA(NPX), NPOL, ICOL(NPX), ILIN(NPX),
     &  NF(4,NDX), NDAT, IFCOL(NDX), IFSYM(NDX), NBL(NPX),
     &  IRETYP(NPX),IMATYP(NPX), NCLEN
C
      COMMON/PPCOM_R/
     &  PLOTAR, CH, CH2,
     &  SIZE,SCRNFR,
     &  CPOLPLF(3,IPTOT), VPOLPLF(3,2), 
     &  XCD,XAL,XOC,
     &  MACH(NPX),REYN(NPX),PTRAT(NPX),ETAP(NPX),
     &  ACRIT(ISX,NPX),XTRIP(ISX,NPX),
     &  CPOL(NAX,IPTOT,NPX),
     &  CPOLSD(NAX,ISX,JPTOT,NPX),
     &  CDLMOD(5,NPX),CDLFIT(5,NPX),VPPARS(6,NPX),DXMREF(NPX),
     &  XYREF(NFX,2,4,NDX),
     &  VERSION
C
      COMMON/PPCOM_C/
     &  FNPOL(NPX), NAME(NPX),
     &  FNREF(NDX), LABREF(NDX), TITLE, CODE, CCLEN, FNAME
C









