#include <stdio.h>

// processor core 
#define sizeisa 256
#define sizemem 252
#define sizebus 4
#define locpc   0          // location of program counter  in core memory
#define locacc  1          // location of oisc accumulator in core memory
#define maxaddr  0xff

typedef unsigned char word;
typedef short oisc;        // each oisc has two, one word args
typedef oisc misa[sizeisa];// complete "mini instruction set architecture"
typedef long rbus[sizebus];// read only bus

typedef struct core {
    misa isa;              // 4096 bits of misa mem 8*2*128
    word mem[sizemem];     // 2016 bits of mem for prog and ram
    rbus bus;              // 32   bits of read-only.  Points to sizebus adjacent nodes.
    char iflag;            // 2    bits download flag tracks wave direction
    char wflag;            // 1    bits download flag tracks if misa or mem is being altered
    char mflag;            // 1    bits download flag adds 1 bit to address space for misa addr
} core;
// ideally the rbus would be contiguous with mem and wouldn't need to be dereferenced
// should be easy to implement in hardware, but hard with software.

#define rows      16
#define accrows   (rows+1)
#define corecount (rows*rows)
core M[corecount];         // matrix of cores
word A[accrows*accrows];   // matrix of accumulators for cores (required for "global shutter")
// Extra rows*4 cores for perimeter outputs
char dy[sizebus] = {-1,  0,  1,  0};
char dx[sizebus] = { 0,  1,  0, -1};
// determines the shape of the matrix

// Currently just subleq.
// If the target location is read-only, only the accumulator is changed.
char OISC(core* c, word I) {
    word a = ((word*) (c->isa+I))[0];
    if (a >= sizemem) {
        c->mem[locacc] = A[c->bus[a-sizemem]] - c->mem[locacc];
    }
    else {
        c->mem[a] = c->mem[locacc] = c->mem[a] - c->mem[locacc];
    }

    // Jump if the accumulator is <= 0
    // In terms of unsigned, should be 0 == acc >= maxaddr/2
    if ((char) c->mem[locacc] <= 0) {
        // Jump to the address ((word*) (c->isa+I))[1] by setting the pc
        c->mem[locpc] = ((word*) (c->isa+I))[1];
        // Ret 1 so the cisc coordinator knows we jumped.
        return 1;
    }
    return 0;
}

#define wavestate 0xff
#define copystate 0xfe
void update(core* c) {
    // if state == wave, update to state = copy
    if      (c->mem[locacc] == wavestate) {c->mem[locacc] = copystate;}
    // if state == copy, download one byte from the bus
    else if (c->mem[locacc] == copystate) {
        word v = A[c->bus[c->iflag]];
        printf("%ld\n", c->bus[c->iflag]);
        c->mem[locacc] = v;
        if      (c->wflag > 1) {
            ((word*) (c->isa+c->mem[locpc]))[c->mflag] = v;
            c->mflag = !c->mflag;
        }
        else if (c->mem[locpc] < sizemem) {c->mem[c->mem[locpc]] = v;}
        c->mem[locpc]++;
    }
    // propagate the wave
    else {
        c->wflag = 0;
        // count # of bus cores currently in the wavestate
        for (char i = 0; i < sizebus; i++) {
            if (A[c->bus[i]] == wavestate) {
                c->wflag++;
                c->iflag = i;
            }
        }
        // transition to wavestate if an adjacent is already in it.
        if (c->wflag) {c->mem[locacc] = wavestate;}
        else          {c->mem[locpc]++;}
    }
}

void CISC(core* c) {
    word I = c->mem[c->mem[locpc]];
    // There will need to be a noop at misa[maxaddr] anyway, as a bookend.
    if (I == maxaddr) {
        update(c);
        return;
    }
        
    char j = 0;
    // Can't jump to the program counter.
    while (((word*) (c->isa+I))[1] != locpc) {j |= OISC(c, I); I += 2;}
    // If a jump occured, j will be 1, and 0 otherwise.
    // We need to only increment the pc if a jump did not occur.
    if (j) {c->mem[locpc]++;}
}

long setneighborhood(int i, int j) {
    int y, x;
    y = i/rows+dy[j];
    x = i%rows+dx[j];
    if      (y == -1)   {return corecount+x;}
    else if (y == rows) {return corecount+3*rows+x;}
    else if (x == -1)   {return corecount+2*y+rows;}
    else if (x == rows) {return corecount+2*y+rows+1;}
    else                {return y*rows+x;}
}
void initialize() {
    int i, j;
    for (i = 0; i < corecount; i++) {
        for (j = 0; j < sizeisa; j++) {M[i].isa[j] = 0;}
        M[i].mem[locpc]         = 2;
        A[i] = M[i].mem[locacc] = 0;
        for (j = 2; j < sizemem; j++) {M[i].mem[j] = 0xff;}
        for (j = 0; j < sizebus; j++) {M[i].bus[j] = setneighborhood(i, j);}
        M[i].iflag = M[i].wflag = M[i].mflag = 0;
    }
}

void loop() {
    unsigned i;
    for (i = 0; i < corecount; i++) {A[i] = M[i].mem[locacc];}
    for (i = 0; i < corecount; i++) {CISC(M+i);}
}

void startup() {
    printf("Starting grobblebox simulation.\n\nInitialize fresh grobblebox? [y/N]\n>");
    if (fgetc(stdin) == 'y') {
        printf("Initializing a new grobblebox...\n");
        initialize();
        printf("Grobblebox initialized.\n\n");
    }
}
void trackwavestate() {
    int w, c;
    w = c = 0;
    for (int i = 0; i < corecount; i++) {
        if (M[i].mem[M[i].mem[locpc]] == maxaddr) {
            if      (M[i].mem[locacc] == wavestate) {
                printf("wavestate at %d %d\n", i/rows, i%rows);
                w++;
            }
        }
    }
    for (int i = 0; i < corecount; i++) {
        if (M[i].mem[M[i].mem[locpc]] == maxaddr) {
            if (M[i].mem[locacc] == copystate) {
                printf("copystate at %d %d\n", i/rows, i%rows);
                c++;
            }
        }
    }
    if (w + c) {printf("%d wavestate grobbles\n%d copystate grobbles\n", w, c);}
}
void runenv(char* filename, void (*program)()) {
    FILE* fp = fopen(filename, "rb");
    fread(M, sizeof(M), 1, fp);
    fclose(fp);

    startup();
    printf("Running grobblebox \"%s\":\n. %d grobbles\n. %ld bytes used\n", filename, corecount, sizeof(M));

    while (1) {program();}

    printf("Finished.  Exiting grobblebox simulation.\n\n");

    fp = fopen(filename, "wb");
    fwrite(M, sizeof(M), 1, fp);
    fclose(fp);
}

void main(int argc, char** argv) {
    if (argc < 2) {
        printf("Not enough args.");
        return;
    }
    // printf("\x1b[2J\x1b[H");
    runenv(argv[1], loop);
}