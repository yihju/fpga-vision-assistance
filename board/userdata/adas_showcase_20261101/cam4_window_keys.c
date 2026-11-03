#define _POSIX_C_SOURCE 200809L
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static int ignore_x_error(Display *d,XErrorEvent *e) {(void)d;(void)e;return 0;}

static int descendant(pid_t pid,pid_t owner) {
    for(int depth=0;pid>1 && depth<64;depth++) {
        if(pid==owner)return 1;
        char name[64],data[4096],state;long parent=0;
        snprintf(name,sizeof(name),"/proc/%ld/stat",(long)pid);
        FILE *f=fopen(name,"r");if(!f)return 0;
        if(!fgets(data,sizeof(data),f)){fclose(f);return 0;}fclose(f);
        char *tail=strrchr(data,')');
        if(!tail || sscanf(tail+1," %c %ld",&state,&parent)!=2 || parent==pid)return 0;
        pid=(pid_t)parent;
    }
    return 0;
}

static Window find_owned(Display *d,Window w,Atom atom,pid_t owner,int depth) {
    if(depth>32)return 0;
    Atom type=0;int format=0;unsigned long count=0,left=0;unsigned char *data=NULL;
    // This board's ximagesink declares the PID property using its own atom
    // instead of CARDINAL. Accept that observed format, not arbitrary types.
    if(XGetWindowProperty(d,w,atom,0,1,False,AnyPropertyType,&type,&format,&count,&left,&data)==Success && data) {
        pid_t pid=(format==32 && count==1 && (type==XA_CARDINAL || type==atom))?(pid_t)((unsigned long *)data)[0]:0;
        XFree(data);
        XWindowAttributes attributes;
        if(pid>1 && descendant(pid,owner) && XGetWindowAttributes(d,w,&attributes) && attributes.map_state==IsViewable)return w;
    }
    Window root,parent,*children=NULL,result=0;unsigned int n=0;
    if(XQueryTree(d,w,&root,&parent,&children,&n)) {
        for(unsigned int i=0;i<n && !result;i++)result=find_owned(d,children[i],atom,owner,depth+1);
        if(children)XFree(children);
    }
    return result;
}

static void listen_tree(Display *d,Window w,int depth) {
    if(depth>32)return;
    XSelectInput(d,w,KeyPressMask|StructureNotifyMask);
    Window root,parent,*children=NULL;unsigned int n=0;
    if(XQueryTree(d,w,&root,&parent,&children,&n)) {
        for(unsigned int i=0;i<n;i++)listen_tree(d,children[i],depth+1);
        if(children)XFree(children);
    }
}

int main(int argc,char **argv) {
    if(argc!=2)return 2;
    char *end=NULL;long value=strtol(argv[1],&end,10);
    if(!end || *end || value<=1)return 2;
    pid_t owner=(pid_t)value;Display *d=XOpenDisplay(NULL);if(!d)return 3;
    XSetErrorHandler(ignore_x_error);
    Atom atom=XInternAtom(d,"_NET_WM_PID",False);Window target=0;
    struct timespec pause={0,20000000};unsigned int ticks=0;
    while(kill(owner,0)==0 || errno==EPERM) {
        if(!target && ticks++%12==0) {
            target=find_owned(d,DefaultRootWindow(d),atom,owner,0);
            if(target) {
                listen_tree(d,target,0);XStoreName(d,target,"Four cameras | Q: Exit");
                XRaiseWindow(d,target);XSetInputFocus(d,target,RevertToParent,CurrentTime);XFlush(d);
                fprintf(stderr,"CAM4_Q_LISTEN window=%lu owner=%ld\n",target,(long)owner);fflush(stderr);
            }
        }
        while(XPending(d)) {
            XEvent event;XNextEvent(d,&event);
            if(event.type==KeyPress) {
                KeySym key=XLookupKeysym(&event.xkey,0);
                if(key==XK_q || key==XK_Q || key==XK_Escape) {
                    puts("q");fflush(stdout);XCloseDisplay(d);return 0;
                }
            }
            if(event.type==DestroyNotify && event.xdestroywindow.window==target)target=0;
        }
        nanosleep(&pause,NULL);
    }
    XCloseDisplay(d);return 0;
}
