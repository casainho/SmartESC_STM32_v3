#define THROTTLEOFFSET 1100
#define THROTTLEMAX 3250
#define BRAKEOFFSET 50
#define BRAKEMAX 190

// speed limits for invividual modes in kph
#define SPEEDLIMIT_ECO 6
#define SPEEDLIMIT_NORMAL 20
#define SPEEDLIMIT_SPORT 50

// motor current limits for invividual modes in mA
// note that hacked firmware allows up to 55amps motor phase current
#define PH_CURRENT_MAX_ECO 20000
#define PH_CURRENT_MAX_NORMAL 30000
#define PH_CURRENT_MAX_SPORT 40000

// motor current limit for regen in mA
#define REGEN_CURRENT 10000

// maximum current for field weakening in mA
#define FIELD_WEAKNING_CURRENT_MAX 0 //max id
