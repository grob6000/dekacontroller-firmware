// definition of types for data structures

typedef enum {Ok, Begin, ZeroM0, ZeroH, SetH, SetM, WaitMark, Error, None} SyncState;
typedef enum {Main, Timezone, Drift, Sync} DisplayMode;
#define MODECOUNT 4

typedef struct {
  DisplayMode displaymode : 2;
  SyncState syncstate : 4;
} ModeStruct;

typedef struct {
  bool displaychange : 1;
  bool messagechange : 1;
} ChangeFlags;

typedef struct {
  bool run_ok : 1;
  bool gps_hastime : 1;
  bool gps_hasfix : 1;
  bool gps_oldfix : 1;
  bool gps_hascomms : 1;
  bool time_drift : 1;
  bool time_error: 1;
} StatusStruct;