//  "EV3ArduinoExtensions"

#define VERSION "1.4"

// Set one of the following to '1' to select which configuration to build for. These each
// represent one of the configurations that we are currently using. More can certainly be added
// as desired. All assume the EV3 interface (since that is the whole reason this code exists).
// Any can have either servos or NeoPixels attached as pins allow. Note that you must also select
// the appropriate Arduino flavor under Tools/Board.
//
// Arduino Uno with the AdaFruit Music Maker Shield
#define UnoWithAdafruitMms 0
// Arduino Uno with the SparkFun Musical Instrument Shield
#define UnoWithSparkFunMis 0
// Pro Micro with the AdaFruit Music Maker Breakout board
#define ProMicroWithAdaFruitMmb 1
// Pro Micro alone
#define ProMicro 0

#if UnoWithAdafruitMms
#define ardAdd 0x04 //I2C address of Arduino when talking to the EV3
// These pins must be defined if both Codec and Synth modes are supported to allow
// the VS1053 to be 'booted' into the respective mode. If only one mode is to be 
// supported, then the GPIO1 pin of the VS1053 should just be tied to the appropriate
// level to have it come up in that mode.
#define vs1053Reset 9      // VS1053 reset pin
#define vs1053Mode 12     // VS1053 mode bit (output)
#define featureCodec 1
// The codec and synthesizer pins are determined by the shield.
#define codecCs 7      // VS1053 chip select pin (output)
#define codecDCs 6     // VS1053 Data/command select pin (output)
#define codecCardCs 4  // Card chip select pin
#define codecDReq 3    // VS1053 Data request, ideally an Interrupt pin
#define featureSynth 1
#define synthPin 2
#define featureServos 0
#define servo0Pin 8
#define servo1Pin 10
#define featureNeoPix 0
#define neoPix0Pin 6
#define neoPix1Pin 8

#elif UnoWithSparkFunMis
#define ardAdd 0x04 //I2C address of Arduino when talking to the EV3
#define featureSynth 1
#define synthPin 3
#define featureServos 1
#define servo0Pin 8
#define servo1Pin 10
#define featureNeoPix 1
#define neoPix0Pin 6
#define neoPix1Pin 8

#elif ProMicroWithAdaFruitMmb
// The main purpose of the Pro Micro with the AdaFruit Music Maker Breakout board is to
// play MP3 and MID files from the SD card. The MP3 playback requires lots of attention 
// from the CPU, but it can deal with simple tasks like running servos. The MID playback
// is much lower data rate.
#define ardAdd 0x04 //I2C address of Arduino when talking to the EV3
#define vs1053Reset 4      // VS1053 reset pin
#define vs1053Mode 20     // VS1053 mode bit (output)
#define featureCodec 1
#define codecCs 18      // VS1053 chip select pin (output)
#define codecDCs 19     // VS1053 Data/command select pin (output)
#define codecCardCs 21  // Card chip select pin
#define codecDReq 7    // VS1053 Data request, ideally an Interrupt pin
#define featureSynth 1
#define synthPin 5
#define allMidiHave2Bytes 1
#define featureServos 1
#define servo0Pin 8
#define servo1Pin 9
#define featureNeoPix 1
#define neoPix0Pin 6
#define neoPix1Pin 10

#elif ProMicro
// The main purpose of the Pro Micro alone is to run the two NeoPixel strips as these
// can be rather time intensive (and thus incompatible with streaming music from the
// SD card). But running the synthesizer (note by note) and the servos are both low
// overhead activities and we allow them to be connected as well.
#define ardAdd 0x05 //I2C address of Arduino when talking to the EV3
#define featureSynth 1
#define synthPin 5
#define featureNeoPix 1
#define neoPix0Pin 6
#define neoPix1Pin 7
#define featureServos 1
#define servo0Pin 8
#define servo1Pin 9

#else
# error "No platform configuration defined"
#endif

// All the Serial.print statements add up. Removing all of them from a full build cuts down almost 4K.
// define VERBOSITY 0 removes all of them.
// define VERBOSITY 1 includes only the highest level statements, like program identification and errors.
// define VERBOSITY 2 includes statements like program configuration and mode changes.
// define VERBOSITY 3 includes everything.
// Keep these levels in mind when adding new statements. Always use one of the _printN and _printlnN 
// variants rather than calling Serial.print[ln]() directly.
#define VERBOSITY 2
#if VERBOSITY >= 1
#define _print1(x) Serial.print(x)
#define _println1(x) Serial.println(x)
#else
#define _print1(x)
#define _println1(x)
#endif
#if VERBOSITY >= 2
#define _print2(x) Serial.print(x)
#define _println2(x) Serial.println(x)
#else
#define _print2(x)
#define _println2(x)
#endif
#if VERBOSITY >= 3
#define _print3(x) Serial.print(x)
#define _println3(x) Serial.println(x)
#else
#define _print3(x)
#define _println3(x)
#endif

#define DIAG_QUEUE 0
#define DIAG_STANDALONE 0

// Memory saving steps.
// In addition to the above VERBOSITY setting, I had to modify the SD library to allow it to compile
// out all of the write capabilities. We had no interest in writing to the SD card, and yet they are 
// linked in because the functions are called by other functions that handle both read and write.

// Note that just not including a header file (via conditional compilation) is insufficient to prevent 
// it from taking up space. Simply having a #include in this source file seems to indicate that the library
// will be linked against, and if it contains any static data it will be linked in. If it contains static
// classes (as does the SD library), then it will also include the code run by the constructor.
#include <SPI.h>
#if !DIAG_STANDALONE
#include <Wire.h>
#endif // !DIAG_STANDALONE
#if featureSynth
#if !featureCodec
#include <SoftwareSerial.h>
#endif // !featureCodec
#endif // featureSynth
#if featureCodec
#include <Adafruit_VS1053.h>
#include <SD.h>
#endif // featureCodec
#if featureServos
#if (servo0Pin == 9 || servo0Pin == 10) && (servo1Pin == 9 || servo1Pin == 10)
// We can use the PWM library if the servos are on pins 9 and 10. The PWM library is more compatible
// with NeoPixels and also saves code (666 bytes) and data space (28 bytes).
#include <Adafruit_TiCoServo.h> 
#define SERVO_CLASS Adafruit_TiCoServo
#else
// Otherwise, we must use the standard Servo library. Note that just not including the header file
// is insufficient to prevent it from taking up space because it defines static data which is always
// linked in if the header is _found_ in the source file.
#include <Servo.h>
#define SERVO_CLASS Servo
#endif
#endif // featureServos
#if featureNeoPix
#include <Adafruit_NeoPixel.h>
#endif // featureNeoPix
#include <SimpleTimer.h>

// The timer is always present to support timing activities of the various features.
SimpleTimer timer;

#if featureCodec
class MidiCodecMode {

    // The music player is used to talk to the VS1053 in codec mode (where it decodes streams
    // of data from .mp3 or .wav or .midi (or many other) file formats. The streams of data are
    // (in this example) read from the SD card and fed to the VS1053 packet by packet (without the
    // arduino doing any processing of the data stream).
    Adafruit_VS1053_FilePlayer musicPlayer;
    bool wasPlaying;

  public:
    // We do not supply the hard-reset pin to the music player because we switch modes using a 
    // soft reset. Our code will make sure it gets a good reset in the setup() routine. This 
    // speeds up the transitions and reduces the clunks on the audio output.
    MidiCodecMode() : musicPlayer( -1 /* vs1053Reset */, codecCs, codecDCs, codecDReq, codecCardCs ), wasPlaying( false ) {}

    bool enter() {
      // We don't use setVS1053Mode because the begin function below does a soft reset. Simply drive
      // the GPIO 1 pin low so it will come up in Codec mode.
      digitalWrite( vs1053Mode, LOW );
      // setVS1053Mode( false );

      // initialise the music player. We do this every time we switch between modes because
      // it performs some initialization required following the reset.
      if (! musicPlayer.begin()) {
        _println1(F("VS1053 not found on SPI"));
        return ( false );
      }

      // While it would certainly make sense that using the DREQ line would be the
      // most efficient way to run it, it has the very nasty problem that it spends
      // a huge amount of time running at interrupt level (like reading data off the
      // the SD card). Since this program is trying to do other things as well, it works
      // better to simply call musicPlayer.feedBuffer() from within the loop (feedBuffer
      // is the function that is called by the interrupt handler as well).
      //
      // This option uses a pin interrupt. But DREQ must be on an interrupt pin.
      // For Uno/Duemilanove/Diecimilla that's Digital #2 or #3
      // See http://arduino.cc/en/Reference/attachInterrupt for other pins
      // if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
      //  _println(F("DREQ pin is not an interrupt pin"));

      musicPlayer.setVolume(20, 20); // default volume, 20 .5 dB steps below maximum.
      //    musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
      return ( true );
    }

    void exit() {
      stop();
    }

    void stop() {
      if ( !musicPlayer.stopped() )
      {
        _println2(F("Stopping playback"));
        musicPlayer.stopPlaying();
        wasPlaying = false;
      }
    }

    void start( const char* filename ) {
      stop();
      if ( musicPlayer.startPlayingFile( filename ) )
      {
        _print2( F("Playing file: ") ); _println2( filename );
        wasPlaying = true;
      }
      else
      {
        _print1( F("Failed to start playing file: ") ); _println1( filename );
      }
    }

    void setVolume( byte volume ) {
      musicPlayer.setVolume( volume, volume );
    }

    bool isPlaying() {
      return ( !musicPlayer.stopped() );
    }

    // This function simply checks to see if the playback of the current file has
    // completed, and updates its state if so.
    void update() {
      musicPlayer.feedBuffer();
      if ( wasPlaying && musicPlayer.stopped() )
      {
        _println2( F("Playback completed") );
        wasPlaying = false;
      }
    }
    Adafruit_VS1053& accessVS1053() { return( musicPlayer ); }
    void softReset() {
      musicPlayer.softReset();
    }
    void writeMidi( byte cmd, byte data1, byte data2 )
    {
      digitalWrite( codecDCs, LOW );
      musicPlayer.spiwrite(0);
      musicPlayer.spiwrite(cmd);
      musicPlayer.spiwrite(0);
      musicPlayer.spiwrite(data1);
      if ( (cmd & 0xF0) <= 0xB0 || (cmd & 0xF0) == 0xE0) {
        musicPlayer.spiwrite(0);
        musicPlayer.spiwrite(data2);
      }
      digitalWrite( codecDCs, HIGH );
    }
    // Since we have vs1053Mode wired to the GPI0 1 pin on the VS1053, we can to a simple test
    // that we can actually control it in both directions. Note that if the function is never 
    // called, it doesn't take up any code space.
    void testVS1053Gpio()
    {
      _println2( F("VS1053 GPIO pin test") );
      Adafruit_VS1053& vs1053( musicPlayer );
      vs1053.GPIO_pinMode( 1, 1 );
      pinMode( vs1053Mode, INPUT );
      vs1053.GPIO_digitalWrite( 1, 1 );
      byte value = digitalRead( vs1053Mode );
      _print2( F("- vs1053 1 -> Arduino ") ); _println2( value );
      vs1053.GPIO_digitalWrite( 1, 0 );
      value = digitalRead( vs1053Mode );
      _print2( F("- vs1053 0 -> Arduino ") ); _println2( value );
      vs1053.GPIO_digitalWrite( 1, 1 );
      value = digitalRead( vs1053Mode );
      _print2( F("- vs1053 1 -> Arduino ") ); _println2( value );
      vs1053.GPIO_pinMode( 1, 0 );
      pinMode( vs1053Mode, OUTPUT );
      digitalWrite( vs1053Mode, 1 );
      value = vs1053.GPIO_digitalRead( 1 );
      _print2( F("- Arduino 1 -> vs1053 ") ); _println2( value );
      digitalWrite( vs1053Mode, 0 );
      value = vs1053.GPIO_digitalRead( 1 );
      _print2( F("- vs1053 0 -> Arduino ") ); _println2( value );
      digitalWrite( vs1053Mode, 1 );
      value = vs1053.GPIO_digitalRead( 1 );
      _print2( F("- Arduino 1 -> vs1053 ") ); _println2( value );
    }
};

// usingAdaFruitMMS - determined during setup whether there is an AdaFruit Music Maker
// Shield attached. If not, it is _assumed_ that there is a SparkFun Music Instrument
// Shield attached (there is no feedback, so the Arduino can't tell if it is there or
// not).
bool usingAdaFruitMMS = false;
bool sdCardPresent = false;
bool mp3CodecMode = false;
MidiCodecMode VS1053_CODEC;

void processCodecPlayTrack( byte cmd, const byte* data )
{
  // Command 2 is the CODEC Play Track.
  // Does nothing if not in CODEC mode. Data is track number. Looks for track in the root of the
  // SD card with the name trackNNN.*, and if found, attempts to play it.
  if( !mp3CodecMode ) {
    byte commandData[] = { 2 };
    processMusicMode( 1, commandData );
  }
  if( mp3CodecMode ) {
    char filename[16];
    strcpy( filename, "track" );
    for ( int i = 5, d = 100; d > 0; ++i, d /= 10 )
      filename[i] = '0' + (data[0] / d) % 10;
    strcpy( filename + 8, ".mp3" );
    if( !SD.exists( filename ) )
      strcpy( filename + 8, ".mid" );
    if( SD.exists( filename ) ) {
      VS1053_CODEC.start( filename );
      if ( !VS1053_CODEC.isPlaying() ) {
        _print1( F("Error: Failed to play ") ); _println1( filename );
      }
    }
    else {
      _print1( F("Error: Track not found ") ); _println1( filename );
    }
  }
}
void processCodecStop( byte cmd, const byte* data )
{
  // Command 5 is the CODEC stop playback command. Data is ignored.
  if( mp3CodecMode ) {
    VS1053_CODEC.stop();
  }
}
#endif // featureCodec

#if featureSynth
bool midiSynthMode = false;
#if !featureCodec
// If we are doing both Codec mode (which assumes the SPI interface to the VS1053) and Synth 
// mode, then we can talk through the SPI/SDI interface to send data to real-time MIDI mode.
SoftwareSerial VS1053_MIDI( 0, synthPin );
#endif // !featureCodec

#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79

#define VS1053_GM1_OCARINA 80
#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

//Plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that data values are less than 127
void talkMIDI( byte cmd, byte data1, byte data2 )
{
#if featureCodec
  VS1053_CODEC.writeMidi( cmd, data1, data2 );
#else
  VS1053_MIDI.write( cmd );
  VS1053_MIDI.write( data1 );

  //Some commands only have one data byte. All cmds less than 0xBn have 2 data bytes
  //(sort of: http://253.ccarh.org/handout/midiprotocol/)
  if ( (cmd & 0xF0) <= 0xB0 || (cmd & 0xF0) == 0xE0)
    VS1053_MIDI.write( data2 );
#endif
}
void processMIDI( byte cmd, const byte* data )
{
  if( !midiSynthMode ) {
    byte commandData[] = { 1 };
    processMusicMode( 1, commandData );
  }
  if( midiSynthMode ) {
    talkMIDI( cmd, data[0], data[1] );
  }
}
#endif // featureSynth

#if featureServos
// This section contains global variables to maintain the state of the two servos.

// SERVO_INTERVAL is the rate, in msec, at which we calculate updated servo positions.
// This is choosen to produce a smooth motion of the servo. Started at the SERVO_TIME_INTERVAL,
// but that is too course and you can see/feel the servo step.
// SERVO_TIME_INTERVAL is the units, in msec, used to specify the interval over which to
// move the servo. This is choosen to give a reasonable range and resolution to the
// specification of the movement. Range is 0 (now!) to 128 * 127 = 16.25 seconds.
#define SERVO_INTERVAL 32
#define SERVO_TIME_INTERVAL 128
// FRAC_BITS is the number of bits used in our fixed-point arithmetic of the calculation of 
// the position at each time interval.
// HALF_FRAC is just the equivelent of one-step-below .5 in our fixed point system.
#define FRAC_BITS 4
#define HALF_FRAC ((1 << (FRAC_BITS-1))-1)

struct servoData {
  SERVO_CLASS servo;
  struct segmentData {
      int8_t endPosition;
      uint8_t travelTime;
  } segments[8];
  uint16_t minPulseWidth;
  uint16_t maxPulseWidth;
  // The current position, in usec (actual is MIN_PULSE_WIDTH + currentPosition).
  int16_t currentPosition;
  // The step on each interval, in fractional usec.
  int16_t stepsPerInterval;
  // The number of intervals until the current leg is completed.
  int16_t intervalsRemaining;
  // 
  // The current leg index into segments.
  byte currentSegment;
  // The number of valid entries in segments.
  byte numSegments;
  // The number of times remaining to repeat segments, or 0xff if forever.
  byte cyclesRemaining;
  // If true, then travelTime is actually 0..100% of max speed (.25 s/60 deg).
  bool specifyDelta;
  bool specifySpeed;
  
  void set( int8_t position ) {
    // currentPosition = ((int16_t) (maxPulseWidth - minPulseWidth) * position / 180) << FRAC_BITS;
    position = constrain( position, -90, 90 );
    currentPosition = map( position, -90, 90, 0, (maxPulseWidth - minPulseWidth) ) << FRAC_BITS;
    uint16_t pulseWidth = minPulseWidth + ((currentPosition + HALF_FRAC) >> FRAC_BITS);
    // _print3( F("set ") ); _println3( pulseWidth );
    servo.write( pulseWidth );
  }
  int8_t get() const {
    int8_t position = map( (currentPosition + HALF_FRAC) >> FRAC_BITS, 0, (maxPulseWidth - minPulseWidth), -90, 90 );
    return( position );
  }
  void stop()
  {
    currentSegment = 0;
    numSegments = 0;
    intervalsRemaining = 0;
    cyclesRemaining = 0;
  }
  void setMode( bool byDelta, bool bySpeed ) {
    specifyDelta = byDelta;
    specifySpeed = bySpeed;
  }
  void addSegment( int8_t position, byte interval ) {
    if( numSegments >= sizeof(segments) / sizeof(segments[0]) ) {
      _println1( F("Error: exceeded max servo segments") );
      return;
    }
    position = constrain( position, -90, 90 );
    segments[numSegments].endPosition = position;
    segments[numSegments].travelTime = interval; // in 128 mSec intervals.
    // _print2( F("add segment ") ); _print2( numSegments ); _print2( F(" end ") ); _print2( segments[numSegments].endPosition ); _print2( F(" time ") ); _println2( segments[numSegments].travelTime );
    ++numSegments;
  }
  void setupSegment()
  {
    int16_t newPosition = map( segments[currentSegment].endPosition, -90, 90, 0, (maxPulseWidth - minPulseWidth) ) << FRAC_BITS;
    int16_t steps = newPosition - currentPosition;
    if( specifySpeed ) {
      // We define 100 to be 60 degrees in 256 mSec. Since the range of pulse widths represents 180 degrees,
      // then 1/3 of the range is the equivelent number of usecs.
      stepsPerInterval = (int16_t) (((maxPulseWidth - minPulseWidth) / 3) << FRAC_BITS) / (256 / SERVO_INTERVAL);
      // At the time of this writing, stepsPerInterval comes out to be 960. If we just do 960 * N / 100,
      // it overflows because 96000 (the maximum) does't fit in a 16-bit signed number. But since we know
      // that it is a constant ending in a zero, we can divide it down once before the multiply, still fit
      // within 16 bits, and 
      stepsPerInterval = ((stepsPerInterval / 10) * segments[currentSegment].travelTime) / 10;
      if( steps < 0 )
        stepsPerInterval = -stepsPerInterval;
      intervalsRemaining = steps / stepsPerInterval;
      if( intervalsRemaining <= 0 )
        intervalsRemaining = 1;
    }
    else {
      int16_t intervals = segments[currentSegment].travelTime * (SERVO_TIME_INTERVAL / SERVO_INTERVAL);
      stepsPerInterval = (steps >= 0 ? steps + (intervals >> 1) : steps - (intervals >> 1)) / intervals; 
      intervalsRemaining = intervals;
    }
    _print3( F("setup segment ") ); _print3( currentSegment ); _print3( F(" steps ") ); _print3( stepsPerInterval ); _print3( F(" intervals ") ); _println3( intervalsRemaining );
  }
  void interval()
  {
    // Is the servo actually running...
    if( intervalsRemaining > 0 ) {
      --intervalsRemaining;
      // Have we reached the end of the current leg?
      if( intervalsRemaining == 0 ) {
        // Then use the exact end position specified (in case of accumulated error).
        currentPosition = map( segments[currentSegment].endPosition, -90, 90, 0, (maxPulseWidth - minPulseWidth) ) << FRAC_BITS;
        // Have we reached the end of the specified segments?
        ++currentSegment;
        if( currentSegment >= numSegments ) {
          currentSegment = 0;
          if( cyclesRemaining != 0xff )
            --cyclesRemaining;
        }
        if( cyclesRemaining != 0 )
          setupSegment();
      }
      else {
        currentPosition += stepsPerInterval;
      }
      uint16_t pulseWidth = minPulseWidth + ((currentPosition + HALF_FRAC) >> FRAC_BITS);
      // _print3( F("int ") ); _println3( pulseWidth );
      servo.write( pulseWidth );
    }
  }
  servoData() : minPulseWidth( MIN_PULSE_WIDTH ), maxPulseWidth( MAX_PULSE_WIDTH ) {}
} servos[2];

void processServos()
{
  for( byte i = 0; i < 2; ++i ) {
    struct servoData& servo( servos[i] );
    servo.interval();
  }
}
void processServoMoveTime( byte cmd, const byte* data )
{
  struct servoData& servo( servos[data[0] & 0x1] );
  servo.stop();
  if( data[2] > 0 ) {
    // data[1] is position in degrees 0..180 (for now). Our servo class works in 
    // degrees from -90..90.
    servo.setMode( false, false );
    int8_t position = (int8_t) (data[1] - 90);
    servo.addSegment( position, data[2] );
    servo.cyclesRemaining = 1;
    servo.setupSegment();
  }
  else {
    servo.set( data[1] );
  }
}
void processServoMoveSpeed( byte cmd, const byte* data )
{
  struct servoData& servo( servos[cmd - 10] );
  servo.stop();
  // data[1] is speed, in percent of maximum from 0..100.
  if( data[1] > 0 ) { // zero speed is nonsensical.
    servo.setMode( false, true );
    // data[0] is position in degrees -90..90 with an offset of 128 so it 
    // works in an unsigned 8-bit value. Our servo class works in degrees
    // from -90..90. 
    int8_t position = (data[0] - 128);
    servo.addSegment( position, data[1] );
    servo.cyclesRemaining = 1;
    servo.setupSegment();
  }
}
void processServoMoveDeltaSpeed( byte cmd, const byte* data )
{
  struct servoData& servo( servos[cmd - 8] );
  servo.stop();
  // data[1] is speed, in percent of maximum from 0..100.
  if( data[1] > 0 ) { // zero speed is nonsensical.
    servo.setMode( false, true );
    // data[0] is degrees to move -128..127 with an offset of 128 so it 
    // works in an unsigned 8-bit value.
    int8_t delta = (data[0] - 128);
    // This is only a partial solution to the 'delta' problem. A real solution
    // will involve storing the delta in the segments. But since all we are 
    // supporting at the moment is a simple (single) move, this works.
    int16_t position = servo.get();
    position += delta;
    position = constrain( position, -90, 90 );
    servo.addSegment( (int8_t) position, data[1] );
    servo.cyclesRemaining = 1;
    servo.setupSegment();
  }
}
void processServoSegment( byte cmd, const byte* data )
{
  struct servoData& servo( servos[data[0] & 0x1] );
  if( cmd == 14 ) {
    servo.stop();
    servo.setMode( false, false );
  }
  servo.addSegment( (int8_t) (data[1] - 90), data[2] > 0 ? data[2] : 1 );
}
void processServoExecute( byte cmd, const byte* data )
{
  struct servoData& servo( servos[data[0] & 0x1] );
  servo.cyclesRemaining = data[1];
  if( servo.cyclesRemaining > 0 ) {
    servo.setupSegment();
  }
}
void processServoReset()
{
  for( byte i = 0; i < 2; ++i ) {
    struct servoData& servo( servos[i] );
    servo.stop();
    servo.set( 0 );
  }
}
#endif // featureServos

#if featureNeoPix
// The configuration will be set at runtime with information from the EV3.
enum NeoPixWalkMode { walkNone, walkPixels, walkFramesBounce, walkFramesRoll };
struct neoPixelData {
  Adafruit_NeoPixel strip;
  uint32_t startColor;
  uint16_t startPosition;
  NeoPixWalkMode walkMode;
  int16_t walkStep;
  uint16_t walkFrame;
  int timerId;
  
  neoPixelData() : startColor( 0 ), startPosition( -1 ), walkMode( walkNone ), walkStep( 0 ), walkFrame( 0 ), timerId( -1 ) {}
} neoPixels[2];

void processNeoPixWalkPixels0()
{
  neoPixelData& npdata( neoPixels[0] );
  npdata.strip.rotatePixels( npdata.walkStep );
  npdata.strip.show();
}
void processNeoPixWalkPixels1()
{
  neoPixelData& npdata( neoPixels[1] );
  npdata.strip.rotatePixels( npdata.walkStep );
  npdata.strip.show();
}
void processNeoPixTimerWalkFrames( byte strip )
{
  neoPixelData& npdata( neoPixels[strip] );
  if( npdata.walkStep > 0 && npdata.walkFrame == npdata.strip.numFrames() - 1 ) {
    if( npdata.walkMode == walkFramesBounce ) {
      npdata.walkStep = -npdata.walkStep;
      npdata.walkFrame += npdata.walkStep;
    }
    else
      npdata.walkFrame = 0;
  }
  else if( npdata.walkStep < 0 && npdata.walkFrame == 0 ) {
    if( npdata.walkMode == walkFramesBounce ) {
      npdata.walkStep = -npdata.walkStep;
      npdata.walkFrame += npdata.walkStep; 
    }
    else
      npdata.walkFrame = npdata.strip.numFrames() - 1;
  }
  else {
    npdata.walkFrame += npdata.walkStep;
  }
  npdata.strip.setFrame( npdata.walkFrame );
  npdata.strip.show();
}
void processNeoPixTimerWalkFrames0() {
  processNeoPixTimerWalkFrames( 0 );
}
void processNeoPixTimerWalkFrames1() {
  processNeoPixTimerWalkFrames( 1 );
}
void processNeoPixConfig( byte cmd, const byte* data )
{
  // Command 20 is NeoPixel Configure. Data0/1 is the number of pixels in the string attached to
  // pin 6, and Data2/3 is the number of pixels in the string attached to pin 7. Should be zero if
  // the string is not present.
  // uint16_t length = ((uint16_t) data[0] << 8 | data[1]);
  byte strip = (data[0] & 0x1);
  uint16_t frames = (data[0] >> 1) + 1;
  neoPixelData& npdata( neoPixels[strip] );
  uint16_t length = data[1];
  byte brightness = data[2];
  if ( length > 0 ) {
    _print2( F("Configure NeoPixel strip ") ); _print2( strip ); 
        _print2( F(" length ") ); _print2(length); _print2( F(" frames ") ); _print2(frames);
        _print2( F(" brightness ") ); _println2(brightness);
    npdata.strip.updateType( NEO_GRB | NEO_KHZ800 );
    // Only update the length if it has changed, since it will realloc unnecessarily.
    if( length != npdata.strip.numPixels() || frames != npdata.strip.numFrames() )
      npdata.strip.updateLength( length, frames );
    npdata.strip.setBrightness( brightness );
    if( npdata.strip.numPixels() == length && npdata.strip.numFrames() == frames ) {
       npdata.strip.begin();
    }
    else {
      _print1( F("Error: failed configuring strip") );
    }
  }
  else {
    _print2( F("Disabling NeoPixel strip ") ); _println2( strip );
    npdata.strip.end();
  }
}
void processNeoPixShow( byte cmd, const byte* data )
{
  byte strip = (data[0] & 0x1);
  uint16_t frame = (data[0] >> 1);
  neoPixelData& npdata( neoPixels[strip] );
  npdata.strip.setFrame( frame );
  npdata.strip.show();
}
void processNeoPixSetStart( byte cmd, const byte* data )
{
  byte strip = (data[0] & 0x1);
  // uint16_t frame = (data[0] >> 1);
  neoPixelData& npdata( neoPixels[strip] );
  npdata.startPosition = data[1];
  npdata.startColor = Adafruit_NeoPixel::Color( data[2], data[3], data[4] );
}
void processNeoPixSetEnd( byte cmd, const byte* data )
{
  byte strip = (data[0] & 0x1);
  uint16_t frame = (data[0] >> 1);
  neoPixelData& npdata( neoPixels[strip] );
  if( npdata.strip.numPixels() ) {
    npdata.strip.setFrame( frame );
    uint16_t endPosition = data[1];
    uint32_t endColor = Adafruit_NeoPixel::Color( data[2], data[3], data[4] );
    if( npdata.startPosition != -1 )
      npdata.strip.setPixelRange( npdata.startPosition, npdata.startColor, endPosition, endColor );
    else
      npdata.strip.setPixelColor( endPosition, endColor );
    npdata.startPosition = -1;
  }
}
void processNeoPixSetAll( byte cmd, const byte* data )
{
  byte strip = (data[0] & 0x1);
  uint16_t frame = (data[0] >> 1);
  neoPixelData& npdata( neoPixels[strip] );
  if( npdata.strip.numPixels() ) {
    npdata.strip.setFrame( frame );
    uint32_t color = Adafruit_NeoPixel::Color( data[2], data[3], data[4] );
    npdata.strip.setPixelRange( 0, color, npdata.strip.numPixels() - 1, color );
    npdata.strip.show();
  }
}
void processNeoPixRotate( byte cmd, const byte* data )
{
  byte strip = (data[0] & 0x1);
  uint16_t frame = (data[0] >> 1);
  neoPixelData& npdata( neoPixels[strip] );
  if( npdata.strip.numPixels() ) {
    npdata.strip.setFrame( frame );
    int16_t shift = (int8_t) data[1];
    npdata.strip.rotatePixels( shift );
    npdata.strip.show();
  }
}
void processNeoPixWalkPixels( byte cmd, const byte* data )
{
  byte strip = (data[0] & 0x1);
  uint16_t frame = (data[0] >> 1);
  neoPixelData& npdata( neoPixels[strip] );
  if( npdata.strip.numPixels() ) {
    if( npdata.timerId >= 0 ) {
      timer.deleteTimer( npdata.timerId );
      npdata.timerId = -1;
    }
    npdata.walkStep = data[1];
    long interval = (long) data[2] << 4;
    if( npdata.walkStep > 0 && interval > 0 ) {
      _print3( F("Starting NeoPixel walk pixels strip ") ); _print3(strip); 
          _print3( F(" step ") ); _print3(npdata.walkStep); _print3( F(" interval ") ); _println3(interval);
      npdata.walkMode = walkPixels;
      npdata.timerId = timer.setInterval( interval, (strip ? processNeoPixWalkPixels1 : processNeoPixWalkPixels0) );
    }
    else {
      npdata.walkMode = walkNone;
      _print3( F("Stopping NeoPixel walk pixels strip ") ); _println3(strip);
    }
  }
}
void processNeoPixWalkFrames( byte cmd, const byte* data )
{
  byte strip = (data[0] & 0x1);
  // uint16_t frame = (data[0] >> 1);
  neoPixelData& npdata( neoPixels[strip] );
  if( npdata.strip.numPixels() ) {
    if( npdata.timerId >= 0 ) {
      timer.deleteTimer( npdata.timerId );
      npdata.timerId = -1;
      npdata.walkMode = walkNone;
    }
    long interval = (long) data[2] << 4;
    if( interval > 0 ) {
      int count = (data[1] & 0x7f);
      _print3( F("Starting NeoPixel walk frames strip ") ); _print3(strip); 
          _print3( F(" count ") ); _print3(count); _print3( F(" interval ") ); _println3(interval);
      npdata.walkMode = (data[1] & 0x40 ? walkFramesBounce : walkFramesRoll);
      // setup the initial conditions so that it immediately rolls or bounces with the first timer interval.
      if( npdata.walkMode == walkFramesBounce ) {
        npdata.walkFrame = 1;
        npdata.walkStep = -1;
      }
      else {
        npdata.walkFrame = npdata.strip.numFrames() - 1;
        npdata.walkStep = 1;
      }
      if( count == 0 )
        npdata.timerId = timer.setInterval( interval, (strip ? processNeoPixTimerWalkFrames1 : processNeoPixTimerWalkFrames0) );
      else
        npdata.timerId = timer.setTimer( interval, (strip ? processNeoPixTimerWalkFrames1 : processNeoPixTimerWalkFrames0), count );
    }
    else {
      _print3( F("Stopping NeoPixel walk frames strip ") ); _println3(strip);
    }
  }
}
void processNeoPixReset()
{
  for( byte i = 0; i < 2; ++i ) {
    neoPixelData& npdata( neoPixels[i] );
    npdata.startColor = 0;
    npdata.startPosition = -1;
    if( npdata.timerId >= 0 ) {
      timer.deleteTimer( npdata.timerId );
      npdata.timerId = -1;
    }
    npdata.walkStep = 0;
    npdata.strip.end();
  }
}
#endif // featureNeoPix

// This function performs a reset of the VS1053 and brings it up in the specified
// mode. If 'midiSynth' is true, then it drives the GPIO 1 pin on the VS1053 high during
// the reset, which causes it to come up in MIDI Synthesizer mode. Otherwise, it drives
// GPIO 1 low during the reset, which brings it up in the Codec mode.
void setVS1053Mode( bool midiSynth )
{
#if defined( vs1053Mode )
#if featureCodec
  // If we are doing both Codec mode (which requires the SPI interface to the VS1053), then
  // we can simply do a soft-reset (which goes through the SPI/SCI interface) to switch it between
  // Codec mode and real-time MIDI mode.
  digitalWrite( vs1053Mode, (midiSynth ? HIGH : LOW) ); // high brings it up in MIDI mode
  VS1053_CODEC.softReset();
  // VS1053_CODEC.testVS1053Gpio();
#elif defined( vs1053Reset )
  //Reset the VS1053. Not really useful at this point.
  digitalWrite( vs1053Reset, LOW );
  digitalWrite( vs1053Mode, (midiSynth ? HIGH : LOW) ); // high brings it up in MIDI mode
  delay(100);
  digitalWrite( vs1053Reset, HIGH );
  delay(100);
#endif
#endif
}
void processMusicMode( byte cmd, const byte* data )
{
  // Command 1 is the Music mode.
  // Data is 0 for none, 1 for SYNTH mode, or 2 for CODEC mode.

  // Turn off any mode that is no longer desired.
#if featureSynth
  if( data[0] != 1 && midiSynthMode ) {
    _println2( F("Turning off MIDI SYNTH mode") );
    // Nothing to do...
    midiSynthMode = false;
  }
#endif // featureSynth
#if featureCodec
  if( data[0] != 2 && mp3CodecMode ) {
    _println2( F("Turning off CODEC mode") );
    VS1053_CODEC.exit();
    mp3CodecMode = false;
  }
#endif // featureCodec

  // Turn on any mode that is now requested.
  if( data[0] == 1 ) {
#if featureSynth
    if( !midiSynthMode ) {
      _println2( F("Turning on MIDI SYNTH mode") );
      // Put into MIDI synthesizer mode.
      setVS1053Mode( true );
#if !featureCodec
      VS1053_MIDI.begin( 31250 ); // MIDI uses a 'strange baud rate'
#endif // !featureCodec

      talkMIDI( MIDI_CHAN_MSG | 0, MIDI_CHAN_BANK, VS1053_BANK_MELODY );
      talkMIDI( MIDI_CHAN_PROGRAM | 0, VS1053_GM1_OCARINA, 0 );
      talkMIDI( MIDI_CHAN_MSG | 0, MIDI_CHAN_VOLUME, 127 );
      midiSynthMode = true;
    }
#else
    _println1( F("Error: MIDI commands not supported by this build") );
#endif // featureSynth
  }
  if( data[0] == 2 ) {
#if featureCodec
    if ( !mp3CodecMode ) {
      if ( usingAdaFruitMMS && sdCardPresent ) {
        _println2( F("Turning on CODEC mode") );
        mp3CodecMode = VS1053_CODEC.enter();
      }
      else if( !usingAdaFruitMMS )
        _println1( F("Error: AdaFruit Music Maker Shield not present") );
      else
        _println1( F("Error: SD card not present") );
    }
#else
    _println1( F("Error: Codec commands not supported by this build") );
#endif
  }
}
void processMusicVolume( byte cmd, const byte* data )
{
  // Command 4 is the MIDI and CODEC Volume command.
  // Data is volume level, from 0 (softest) to 127 (loudest). Can be used for both
  // SYNTH mode and CODEC mode. In SYNTH mode, it sets the volume for all channels (there
  // is no global volume). For individual channel control, use the MIDI blocks MIDI Cmd to send (0xB0 | chan), 0x07, volume.
  byte volume = data[0];
  if( volume > 127 )
    volume = 127;
#if featureSynth
  if( midiSynthMode ) {
    for ( int chan = 0; chan < 16; ++chan )
      talkMIDI( MIDI_CHAN_MSG | chan, MIDI_CHAN_VOLUME, volume );
  }
#endif // featureSynth
#if featureCodec
  if( mp3CodecMode ) {
    VS1053_CODEC.setVolume( 127 - volume );
  }
#endif // featureCodec
}
void processReset( byte cmd, const byte* data )
{
  // Command 127 is the 'restore default state' command. It basically sets everything back to
  // the power on defaults so that the EV3 program is starting from a known state.
  _println2( F("Restoring default state") );
#if featureNeoPix
  processNeoPixReset();
#endif // featureNeoPix
#if featureServos
  processServoReset();
#endif // featureServos
#if featureSynth
  if ( midiSynthMode )
  {
    // Nothing to do...
    midiSynthMode = false;
  }
#endif // featureSynth
#if featureCodec
  if ( mp3CodecMode )
  {
    VS1053_CODEC.exit();
    mp3CodecMode = false;
  }
#endif

  // Originally did this only during setup, but the Pro Micro boards don't talk over serial
  // while running setup, so you never see it. Now I dump it out on each reset default state.
  showCapabilities();
}
void processTest( byte cmd, const byte* data )
{
  _print3( F("Test cmd ") ); _print3( cmd );
  for( int i = 0; i < 5; i++ ) {
    _print3( F(" ") ); _print3( data[i] );
  }
  _println3( F("") );
}
void processUnrecognized( byte cmd, const byte* data )
{
  _print1( F("Error: Unrecognized command ") ); _println1(cmd);
}

// A simple queue for commands received from the EV3.
// cmdQueueWrite is the next index at which a byte will be written.
// cmdQueueRead is the next index at which a byte will be read.
// So cmdQueueRead == cmdQueueWrite means the queue is empty.
// and cmdQueueRead == (cmdQueueWrite + 1) % sizeof(cmdQueue) means the queue is full.
// It's capacity in bytes is one less than the specified size of queue.
struct xferQueue {
  byte queue[32];
  byte writeIdx;
  byte readIdx;
  bool overflowOccured;
  xferQueue() : writeIdx( 0 ), readIdx( 0 ), overflowOccured( false ) {}
  inline void reset() {
    writeIdx = readIdx = 0;
  }
  inline bool empty() {
    return ( readIdx == writeIdx );
  }
  inline bool full() {
    return ( readIdx == (writeIdx + 1) % sizeof(queue) );
  }
  inline byte at( byte idx ) {
    return( queue[idx] );
  }
  inline byte peek_front() {
    return( queue[readIdx] );
  }
  inline byte pop_front() {
    byte data = queue[readIdx];
    readIdx = advance( readIdx );
    return( data );
  }
  inline void push_back( byte data ) {
    if( !full() ) {
      queue[writeIdx] = data;
      writeIdx = advance( writeIdx );
    }
    else
      overflowOccured = true;
  }
  inline byte advance( byte idx ) {
    return ( (idx + 1) % sizeof(queue) );
  }
  inline byte bytes() {
    if ( writeIdx >= readIdx )
      return ( writeIdx - readIdx );
    else
      return ( writeIdx + sizeof(queue) - readIdx );
  }
  inline byte free() {
    return( sizeof(queue) - 1 - bytes() );
  }
  inline bool overflow() {
    bool data = overflowOccured;
    overflowOccured = false;
    return( data );
  }
} cmdQueue; // , reqQueue;
volatile bool cmdProcessing = true;
#if DIAG_QUEUE
int readRequests = 0;

void processCmdQueueCheck()
{
  _print2( F("CmdQueue bytes ") ); _print2( cmdQueue.bytes() ); _print2( F(" available ") ); _print2( Wire.available() ); _print2( F(" reads " ) ); _print2( readRequests ); _print2( F(" status ") ); _println2( genStatusByte() );
  readRequests = 0;
}
#endif // DIAG_QUEUE

void showCapabilities()
{
  _println1( F("EV3 Arduino Extensions " VERSION " - " __DATE__ " " __TIME__) );
#if DIAG_STANDALONE
  _println1( F("Standalone Mode") );
#endif // DIAG_STANDALONE

#if featureCodec
  _println2( F("Codec support for AdaFruit Music Maker Shield using SPI.") );
  if( usingAdaFruitMMS ) {
    _println2( F("- detected AdaFruit Music Maker Shield") );
  }
  if ( sdCardPresent ) {
    _println2( F("- detected SD card") );
  }
#endif // featureCodec

#if featureSynth
#if featureCodec
  _println2( F("MIDI synthesizer support using SPI/SDI") );
#else
  _print2( F("MIDI synthesizer support using serial on pin ") ); _println2( synthPin );
#endif
#endif // featureSynth

#if defined( vs1053Mode )
  _print2( F("Switching music modes requires GPIO1 to pin ") ); _println2( vs1053Mode );
#endif // defined( vs1053Mode )

#if featureServos
  _print2( F("Servo support for servos on pin ") ); _print2( servo0Pin ); _print2( F( " and pin ") ); _println2( servo1Pin );
#endif // featureServos

#if featureNeoPix
  _print2( F("Led support for strips on pin ") ); _print2( neoPix0Pin ); _print2( F(" and pin ") ); _println2( neoPix1Pin );
#endif // featureNeoPix
}

void setup()
{
#if VERBOSITY
  Serial.begin(115200);
#endif // VERBOSITY

#if !DIAG_STANDALONE
  Wire.begin( ardAdd );
  Wire.onReceive( receiveData );
  Wire.onRequest( requestData );
#else
  timer.setTimeout( 1000, processTestScript );
#endif
#if DIAG_QUEUE
  timer.setInterval( 10000, processCmdQueueCheck );
#endif // DIAG_QUEUE

#if featureNeoPix
  // This preceedes the VS1053 initialization (which can take a half second) just so that the
  // lights are turned off as soon as possible if present.
  neoPixels[0].strip.setPin( neoPix0Pin );
  neoPixels[0].strip.clearStrip();
  neoPixels[1].strip.setPin( neoPix1Pin );
  neoPixels[1].strip.clearStrip();
  processNeoPixReset();
#endif // featureNeoPix

  // The vs1053Mode pins in used to change the mode of the VS1053, so must be output.
#if defined( vs1053Mode )
  pinMode( vs1053Mode, OUTPUT );
#endif
#if defined( vs1053Reset )
  // If we have a reset line to the VS1053, then give it a nice reset.
  digitalWrite( vs1053Reset, LOW );
  pinMode( vs1053Reset, OUTPUT );
  delay( 100 );
  digitalWrite( vs1053Reset, HIGH );
  // delay( 100 ) not necessary because the .enter() below has one.
#endif // defined( vs1053Reset ) && vs1053Reset != -1

#if featureCodec
  // If we can enter CODEC mode, then we interpret that to mean that we are running with
  // the AdaFruit Music Maker Shield.
  usingAdaFruitMMS = VS1053_CODEC.enter();
  // VS1053_CODEC.exit(); not necessary since we know we haven't played anything.
  if( usingAdaFruitMMS ) {
    // It is possible that a different shield is present that supports the SD card, but for
    // now we only look if the AdaFruit shield is present.
    sdCardPresent = SD.begin( codecCardCs );
  }
#endif // featureCodec

#if featureSynth
  // If necessary, the pin of the SoftwareSerial object can be changed like this. For 
  // now, we are defining it at compile time so this is unnecessary.
  // Switch the MIDI serial port to pin 3 for the SparkFun Shield.
  // VS1053_MIDI = SoftwareSerial( 0, synthPin );
#endif // featureSynth

#if featureServos
  servos[0].servo.attach( servo0Pin, MIN_PULSE_WIDTH + 130, MAX_PULSE_WIDTH - 130 );  // attaches the servo object to the appropriate pin.
  servos[1].servo.attach( servo1Pin, MIN_PULSE_WIDTH + 130, MAX_PULSE_WIDTH - 130 );
  processServoReset();
  timer.setInterval( SERVO_INTERVAL, processServos );
#endif // featureServos

  showCapabilities();

  cmdProcessing = false;
}

// Recall that this function, loop, gets called continuously.
void loop()
{
  processCmdQueue();

#if featureCodec
  if ( mp3CodecMode )
    VS1053_CODEC.update();
#endif // featureCodec

  timer.run();
}

#if !DIAG_STANDALONE
// This routine is called (in the context of an interrupt) when data has been received
// by the wire/I2C interface. The code suggests that it will be called when the end of a
// possibly multi-byte sequence is received. Not so sure how this will work...
void receiveData(int howMany)
{
  for ( int i = 0; i < howMany; i++ )
  {
    byte data = Wire.read();
    cmdQueue.push_back( data );
  }
}
void requestData()
{
//  if( !reqQueue.empty() ) {
//    Wire.write( reqQueue.pop_front() );
//  }
//  else {  
    Wire.write( genStatusByte()  );
#if DIAG_QUEUE
    ++readRequests;
#endif // DIAG_QUEUE
//  }
}
#endif // !DIAG_STANDALONE

byte genStatusByte() 
{
    byte data = 0x80;
    if( cmdProcessing || !cmdQueue.empty() )
      data |= 0x1;
#if featureCodec
    if( mp3CodecMode )
      data |= 0x2;
#endif // featureCodec
#if featureSynth
    if( midiSynthMode )
      data |= 0x4;
#endif // featureSynth
#if featureCodec
    if( usingAdaFruitMMS )
      data |= 0x10;
    if( sdCardPresent )
      data |= 0x20;
    if( VS1053_CODEC.isPlaying() )
      data |= 0x40;
#endif // featureCodec
  return data;
}

// This table is a map from a command to a function to process that command. I have also
// included a 'numBytes' with the thought that in the future commands could be of different
// lengths, but right now they are all 2.
//
// Note the PROGMEM pushes the table into Flash, which saves a bit of RAM but requires that
// the table be accessed using the pgm_read_* commands as shown in the loop below.
typedef void (*commandFunction)( byte cmd, const byte* data );
static const struct {
  byte cmd;
  byte dataBytes;
  commandFunction func;
} commandToFuncMap[] PROGMEM = {
  { 1, 1, processMusicMode },
  { 4, 1, processMusicVolume },
#if featureCodec
  { 2, 1, processCodecPlayTrack },
  { 5, 1, processCodecStop },
#endif // featureCodec
#if featureServos
  { 8, 2, processServoMoveDeltaSpeed }, // servo is low bit of command.
  { 9, 2, processServoMoveDeltaSpeed }, // servo is low bit of command.
  { 10, 2, processServoMoveSpeed }, // servo is low bit of command.
  { 11, 2, processServoMoveSpeed }, // servo is low bit of command.
  { 13, 3, processServoMoveTime },
  { 14, 3, processServoSegment },
  { 15, 3, processServoSegment },
  { 16, 3, processServoExecute },
#endif // featureServos
#if featureNeoPix
  { 20, 3, processNeoPixConfig },
  { 21, 1, processNeoPixShow },
  { 24, 5, processNeoPixSetStart },
  { 25, 5, processNeoPixSetEnd },
  { 26, 5, processNeoPixSetAll },
  { 30, 3, processNeoPixRotate },
  { 31, 3, processNeoPixWalkPixels },
  { 32, 3, processNeoPixWalkFrames },
#endif // featureNeoPix
  { 125, 3, processTest },
  { 126, 5, processTest },
  { 127, 0, processReset },
};

void processCmdQueue()
{
  // _print("write: "); _print(cmdQueueWrite); _print(", read: "); _println(cmdQueueRead);

  if ( cmdQueue.overflow() ) {
    _println1( F("Error: Command queue overflow, likely command errors") );
  }
  
  while( !cmdQueue.empty() ) {
    byte cmd = cmdQueue.peek_front();
    commandFunction cmdFunc = processUnrecognized;
    byte dataBytes = 0;
#if featureSynth
    // handle all of the MIDI command separately because they cover a large range of values 
    // (everything with the msb set) and can have 1 or 2 data bytes.
    if( (cmd & 0x80) != 0 ) {
      cmdFunc = processMIDI;
#if allMidiHave2Bytes
      dataBytes = 2;
#else
      dataBytes = 1;
      if ( (cmd & 0xF0) != 0xC0 && (cmd & 0xF0) != 0xD0 )
        dataBytes = 2;
#endif
    }
    else {
#endif // featureSynth
      for ( int i = 0; i < sizeof(commandToFuncMap) / sizeof(commandToFuncMap[0]); ++i )
      {
        byte mapCmd = pgm_read_byte_near( &commandToFuncMap[i].cmd );
        // _print( "i " ); _print( i ); _print( " cmd "); _println( mapCmd );
        if ( cmd == mapCmd )
        {
          cmdFunc = (commandFunction) pgm_read_word_near( &commandToFuncMap[i].func );
          dataBytes = pgm_read_byte_near( &commandToFuncMap[i].dataBytes );
          break;
        }
      }
#if featureSynth
    }
#endif // featureSynth

    if( cmdQueue.bytes() >= 1 + dataBytes ) {
      cmdProcessing = true;
      // Now that we know we have all the bytes, we can update our read pointer.
      cmdQueue.pop_front();
      byte data[7];
      for ( byte i = 0; i < dataBytes; ++i )
        data[i] = cmdQueue.pop_front();
      for ( byte i = dataBytes; i < sizeof(data); ++i )
        data[i] = 0;

#if VERBOSITY >= 3
      _print3( F("Processing cmd ") ); _print3( cmd );
      for( byte i = 0; i < dataBytes; ++i ) {
        _print3( F(", ") ); _print3( data[i] );
      }
      _println3( F("") );
#endif // VERBOSITY >= 3

      // now that we have the data bytes collected, invoke the processing function.
      (*cmdFunc)( cmd, data );
  
      cmdProcessing = false;
    }
    else
    {
      // At the moment, I cannot send six bytes at once from the EV3. So I tollerate getting them
      // as 2+4.
      if( (cmdQueue.bytes() == 2 || cmdQueue.bytes() == 4) && dataBytes >= 3 ) {
        break;
      }
      // data byte has not arrived yet. Process it next time. Why don't we wait 'forever' for it to 
      // arrive? Because sometimes we get garbage over the wire, and if it looks like a command then
      // it gets stuck. This way, if the rest of the command does not show up fairly quickly, then
      // we discard it.
      static byte s_tries = 0;
      ++s_tries;
      if ( s_tries > 3 )
      {
        _print1( F("Error: Incomplete command ") ); _println1(cmd);
        cmdQueue.pop_front();
        s_tries = 0;
      }
      break; // out of the while loop
    }
  } //   while( !cmdQueue.empty() )
}

#if DIAG_STANDALONE
// In DIAG_STANDALONE, the cmdQueue is filled from the following 'script'.
// For each entry, it waits the specified amout of time, and then places the supplied
// bytes into the queue.

static const struct {
  uint16_t delay;
  byte dataBytes;
  byte bytes[7];
} testScript[] PROGMEM = {
  { 100,   0, { 127, 0 } },             // Reset
#if featureNeoPix
  { 100,   3, { 20, 0, 8, 127, 0 } },   // Configure Leds 0
  { 0,     3, { 20, 1, 8, 127, 0 } },   // Configure Leds 1
  { 0,     5, { 24, 0, 0, 127, 10, 10, 0 } },
  { 0,     5, { 25, 0, 7, 10, 10, 127, 0 } },
  { 0,     3, { 31, 0, 1, 32, 0 } },    // Walk Leds
#endif // featureNeoPix
#if featureServos
  { 100,   3, { 14, 0, 160, 20, 0 } },   // Servo start path
  { 0,     3, { 15, 0, 160, 10, 0 } },   // Servo add segment
  { 0,     3, { 15, 0,  90, 40, 0 } },   // Servo add segment
  { 0,     3, { 15, 0,  20, 20, 0 } },   // Servo add segment
  { 0,     3, { 15, 0,  20, 10, 0 } },   // Servo add segment
  { 0,     3, { 15, 0,  90, 40, 0 } },   // Servo add segment
  { 0,     3, { 16, 0, 127, 0 } },       // Servo execute path
#endif // featureServos
#if featureCodec
  { 100,   1, { 1, 2, 0 } },            // Track mode
  { 0,     1, { 2, 1, 0 } },            // Play track 1
#if featureNeoPix
  { 100,   5, { 26, 1, 0, 10, 31, 10 } }, // Leds 1 to green
  { 0,     5, { 25, 1, 0, 0, 0, 0, 0 } }, // Except Led 0
  { 0,     1, { 21, 1, 0 } },           // Show Led 1
#endif // featureNeoPix
  { 20000, 1, { 5, 0, 0 } },            // Stop track
#if featureServos
  { 0,     3, { 16, 0, 0, 0 } },        // Servo stop path
#endif // featureServos
  { 100,   1, { 2, 2, 0 } },            // Play track 2
#if featureNeoPix
  { 100,   5, { 26, 1, 0, 10, 63, 10 } }, // Leds 1 to green
  { 0,     5, { 25, 1, 1, 0, 0, 0, 0 } }, // Except Led 1
  { 0,     1, { 21, 1, 0 } },           // Show Led 1
#endif // featureNeoPix
  { 20000, 1, { 5, 0, 0 } },
#if featureServos
  // { 100,   3, { 13, 0, 45, 0, 0 } },   // Servo move
  { 100,   2, { 10, -45 + 128, 100, 0 } },   // Servo move
#endif // featureServos
  { 100,   1, { 2, 3, 0 } },            // Play track 3
#if featureNeoPix
  { 100,   5, { 26, 1, 0, 10, 95, 10 } }, // Leds 1 to green
  { 0,     5, { 25, 1, 2, 0, 0, 0, 0 } }, // Except Led 2
  { 0,     1, { 21, 1, 0 } },           // Show Led 1
#endif // featureNeoPix
  { 20000, 1, { 5, 0, 0 } },
#if featureServos
  // { 100,   3, { 13, 0, 135, 0, 0 } },    // Servo move
  // { 100,   2, { 10, 45 + 128, 50, 0 } },    // Servo move
  { 100,   2, { 8, 90 + 128, 50, 0 } },    // Servo move
#endif // featureServos
  { 100,   1, { 2, 4, 0 } },            // Play track 4
#if featureNeoPix
  { 100,   5, { 26, 1, 0, 10, 127, 10 } }, // Leds 1 to green
  { 0,     5, { 25, 1, 3, 0, 0, 0, 0 } }, // Except Led 3
  { 0,     1, { 21, 1, 0 } },           // Show Led 1
#endif // featureNeoPix
  { 20000, 1, { 5, 0, 0 } },
#endif // featureCodec
#if featureNeoPix
  { 0,     3, { 31, 0, 0, 0, 0 } },     // Stop Leds
  { 100,   5, { 24, 0, 0, 10, 127, 10, 0 } },
  { 0,     5, { 25, 0, 7, 10, 10, 127, 0 } },
  { 0,     5, { 26, 1, 0, 127, 10, 10 } }, // Leds 1 to red
#endif // featureNeoPix
#if featureServos
  // { 100,   3, { 13, 0, 90, 0, 0 } },    // Servo move
  // { 100,   2, { 10, 0 + 128, 25, 0 } },    // Servo move
  { 100,   2, { 8, -45 + 128, 25, 0 } },    // Servo move
#endif // featureServos
#if featureSynth
  { 100,   1, { 1, 1, 0 } },            // Note mode
  { 500,   2, { 153, 36, 127, 0 } },    // BdB
  { 60,    2, { 153, 45, 127, 0 } },    
  { 60,    2, { 153, 49, 127, 0 } },    
#if featureNeoPix
  { 0,     3, { 30, 0, -1, 0, 0 } },    // Rotate Leds
#endif // featureNeoPix
  { 500,   2, { 153, 36, 100, 0 } },    
  { 60,    2, { 153, 45, 100, 0 } },    
  { 60,    2, { 153, 49, 100, 0 } }, 
#if featureNeoPix
  { 0,     3, { 30, 0, -2, 0, 0 } },    // Rotate Leds
#endif // featureNeoPix
  { 500,   2, { 153, 36, 75, 0 } },    
  { 60,    2, { 153, 45, 75, 0 } },    
  { 60,    2, { 153, 49, 75, 0 } }, 
#if featureNeoPix
  { 0,     3, { 30, 0, -4, 0, 0 } },    // Rotate Leds
#endif // featureNeoPix
#endif // featureSynth
  { 500,   1, { 1, 0, 0 } },            // No music
};
byte testScriptIndex = 0;

void processTestScript()
{
  uint16_t delay = 0;
  do {
    byte dataBytes = pgm_read_byte_near( &testScript[testScriptIndex].dataBytes );
    // Not enough space, so bail out of this iteration. Note it is ok if delay == 0 in the 
    // setTimeout below - our routine will just be called again in the next run.
    if( dataBytes + 1 > cmdQueue.free() )
      break;
    for( byte i = 0; i <= dataBytes; ++i )
      cmdQueue.push_back( pgm_read_byte_near( &testScript[testScriptIndex].bytes[i] ) );
    ++testScriptIndex;
    if( testScriptIndex >= sizeof( testScript ) / sizeof( testScript[0] ) )
      testScriptIndex = 0;
    delay = pgm_read_word_near( &testScript[testScriptIndex].delay );
  } while( delay == 0 );
  timer.setTimeout( delay, processTestScript );
}
#endif // DIAG_STANDALONE

