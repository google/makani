C---- Global variables for AVL plotting routines
C
C
      LOGICAL LPLOT, LCURS, LHID, LCREV
      COMMON /PLT_I/  LPLOT, LCURS, LHID, LCREV,
     &                IDEV, IDEVH, IDEVM, IPSLU,
     &                NCOLORS
C
      COMMON /PLT_R/  SCRNFRAC, SIZE, PLOTAR,
     &                XMIN,  XMAX,  YMIN, YMAX,
     &                XOFF,  YOFF,  SF,   CH,
     &                XMARG,YMARG,
     &                XPAGE,YPAGE,
     &                XWIND,YWIND,
     &               PMARG,CPFAC, HNFAC, ENFAC,
     &               AZIMOB,ELEVOB,TILTOB,ROBINV
C
      PARAMETER (NHAXIS = 12)  ! number of segments forming x,y,z arrowheads
      PARAMETER (NHX = 2*NHAXIS+1)
      COMMON /AXES_I/
     &    NHEAD ,
     &    NLINAX

      COMMON /AXES_R/
     &    ARWLEN,
     &    RHEAD,
     &    RHEADP,
     &    UAXDIR(3,3),
     &    UAXDIRP(3,3),
     &    UAXARW(3,2,NHX,3),
     &    UAXARWP(3,2,NHX,3)
