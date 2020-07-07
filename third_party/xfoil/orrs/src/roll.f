
      program roll
c----------------------------------------------------------------
c     Computes mean profile and Reynolds stress tensor components
c     of Lamb vortex "roller" street.
c----------------------------------------------------------------
      parameter (nx=100,ny=200)
      real x(nx,ny), y(nx,ny), u(nx,ny), v(nx,ny), w(nx,ny)
      real uavg(ny), yavg(ny)
      real uu(ny), vv(ny), uv(ny), qq(ny)
c
      St = 0.19
ccc      Wmax = 4.45
      Wmax = 0.90
c
      vfrac = 0.75
c
      pi = 4.0*atan(1.0)
c
      xmin = -0.5*pi/St
      xmax =  0.5*pi/St
c
      ymin = -0.75*pi/St
      ymax =  0.75*pi/St
c
      do i=1, nx
        do j=1, ny
           x(i,j) = xmin + (xmax-xmin)*float(i-1)/float(nx-1)
           y(i,j) = ymin + (ymax-ymin)*float(j-1)/float(ny-1)
c
           usum = 0.
           vsum = 0.
c
           do k = -100, 100
             xb = x(i,j) + float(k)*pi/St
             yb = y(i,j)
             rsq = xb**2 + yb**2
c
             arg = Wmax*St*rsq
             arg = min( arg , 30.0 )
             ex1 = 1.0 - exp(-arg)
c
             usum = usum + yb/rsq * ex1
             vsum = vsum - xb/rsq * ex1
           enddo
c
           u(i,j) = usum * 0.5/St
           v(i,j) = vsum * 0.5/St
c
         enddo
       enddo
c
       do j=1, ny
         yavg(j) = y(1,j)
         uavg(j) = 0.
         do i=1, nx-1
           uavg(j) = uavg(j) + u(i,j)/float(nx-1)
         enddo
       enddo
c
       do i=1, nx
         do j=1, ny
           u(i,j) = vfrac*u(i,j) + (1.0-vfrac)*uavg(j)
           v(i,j) = vfrac*v(i,j)
         enddo
       enddo
c
c
       do i=2, nx-1
         do j=2, ny-1
           dx = x(i+1,j) - x(i-1,j)
           dy = y(i,j+1) - y(i,j-1)
           dv = v(i+1,j) - v(i-1,j)
           du = u(i,j+1) - u(i,j-1)
c
           w(i,j) = du/dy - dv/dx
         enddo
       enddo
c
c
       theta = 0.0
       do j=1, ny-1
         ua = (uavg(j+1) + uavg(j))*0.5  +  0.5
         dy =  yavg(j+1) - yavg(j)
         theta = theta + (1.0 - ua)*ua*dy
       enddo
c
       write(*,*) 'Theta = ', theta
c
       do j=1, ny
         uu(j) = 0.
         vv(j) = 0.
         uv(j) = 0.
         do i=1, nx-1
           up = u(i,j) - uavg(j)
           vp = v(i,j)
           uu(j) = uu(j) + up*up / float(nx-1)
           vv(j) = vv(j) + vp*vp / float(nx-1)
           uv(j) = uv(j) + up*vp / float(nx-1)
         enddo
         qq(j) = uu(j) + vv(j)
       enddo
c
       qint = 0.
       do j=2, ny-1
         qint = qint + uavg(j)*(uu(j) + vv(j)) / float(ny-2)
       enddo
c
c
       idev = 1
       size = 7.0
       ncolor = 64
       ch = 0.01
c
       XOFF = 0.
       YOFF = 0.
       GWT = 0.8 / (YMAX-YMIN)
c
       call plinitialize
       call colorspectrumhues(ncolor,'ROYGCB')
c
c
       call plopen(0.8,0,idev)
       call newfactor(size)
c
       call plot(0.1,0.1,-3)
       call plot(-xmin*GWT,-ymin*GWT,-3)
c
       do ic=1, ncolor
         wcon = Wmax * float(ic-1)/float(ncolor-1)
         call newcolor(-ic)
         call CONTGRID(NX,NY,NX,NY,X,Y,W,WCON,XOFF,YOFF,GWT,GWT)
       enddo
c
       call newcolorname('black')
c
       ydel = 2.0
       y1 = -12.0
       y2 =  12.0
c
c------------------
       call plot(xmax*GWT+0.1,0.0,-3)
c
       uwt = 0.3
c
       udel = 0.2
       u1 = 0.
       u2 = 1.0
c
       call yaxis(0.0,y1*gwt,(y2-y1)*gwt,ydel*gwt,y1,ydel,ch,-2)
       call xaxis(0.0,0.0,-uwt*(u2-u1),uwt*udel,u1,udel,ch,1)
c
       call xyline(ny,uavg,yavg,-0.5,uwt,0.0,gwt,1)
c
c------------------
       call plot(uwt+0.1,0.0,-3)
c
       twt = 3.0
c
       tdel = 0.02
       t1 = 0.
       t2 = 0.1
       call yaxis(0.0,y1*gwt,(y2-y1)*gwt,ydel*gwt,y1,ydel,ch,-2)
       call xaxis(0.0,0.0,-twt*(t2-t1),twt*tdel,t1,tdel,ch,-2)
c
       call xyline(ny,qq,yavg,0.0,twt,0.0,gwt,1)
       call xyline(ny,uu,yavg,0.0,twt,0.0,gwt,2)
       call xyline(ny,vv,yavg,0.0,twt,0.0,gwt,3)
       call xyline(ny,uv,yavg,0.0,10.0*twt,0.0,gwt,4)
c
c------------------

       call plflush
       pause
       call plend
c
c
       call plopen(0.8,0,idev)
       call newfactor(size)
c
       call plot(0.1,0.1,-3)
       call plot(-xmin*GWT,-ymin*GWT,-3)
c
       do ic=1, ncolor
         Ucon = -2.0 + 4.0*float(ic-1)/float(ncolor-1)
         call newcolor(-ic)
         call CONTGRID(NX,NY,NX,NY,X,Y,U,UCON,XOFF,YOFF,GWT,GWT)
       enddo
c
       call plflush
       pause
c
c
       call plclose
       stop
       end

