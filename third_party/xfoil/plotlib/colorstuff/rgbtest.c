
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rgbtbl.h"

main(argc,argv)
char **argv;
int argc;
{

int IC,ic,ncols;
int i,n,ired,igrn,iblu;
char teststr[50], c;

IC = sizeof(Colordef);
ic = sizeof(colordef);
ncols = ic/IC;

printf("Colordef %d\n",IC);
printf("colordef %d\n",ic);
printf("#entries %d\n",ncols);

printf("\nEnter color string: ");

/*if ((c=fgetc(stdin)) != '\n')
ungetc(c,stdin); */

fgets(teststr,50,stdin);
printf("strlen = %d\n",strlen(teststr));

if ((n=strlen(teststr)) != 0) {
    while(n > 0 && teststr[--n] == '\n')
      teststr[n] = '\0'; } 

/*fscanf(stdin,"%s",teststr);*/
printf("\nTest string = %s\n",teststr);

for (i=0; i < ncols; i++) {
  if(!strcasecmp(colordef[i].name,teststr)) {

   printf("test  string %s found at %d\n",teststr,i); 
   printf("found string %s code %d\n",colordef[i].name,colordef[i].rgb); 

   ired = 0x000000ff & (colordef[i].rgb >> 16);
   igrn = 0x000000ff & (colordef[i].rgb >> 8); 
   iblu = 0x000000ff & (colordef[i].rgb); 

   printf("red = %d\ngrn = %d\nblu = %d\n",ired,igrn,iblu);
  }
}

}


