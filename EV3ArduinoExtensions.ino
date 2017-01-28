//  "EV3ArduinoExtensions"

// Set one of the following to '1' to select which configuration to build for. These each
// represent one of the configurations that we are currently using. More can certainly be added
// as desired. All assume the EV3 interface (since that is the whole reason this code exists).
// Any can have either servos or NeoPixels attached as pins allow.
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
#define featureNeoPix 1
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
#define featureServos 1
#define servo0Pin 8
#define servo1Pin 9

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

#include <SPI.h>
#include <Wire.h>
#if featureSynth
#include <SoftwareSerial.h>
#endif // featureSynth
#if featureCodec
#include <Adafruit_VS1053.h>
#include <SD.h>
#endif // featureCodec
#if featureServos
#include <Servo.h>
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
    MidiCodecMode() : musicPlayer( vs1053Reset, codecCs, codecDCs, codecDReq, codecCardCs ), wasPlaying( false ) {}

    bool enter() {
      // We don't use resetVS1053 because the begin function below will do a reset. Simply drive
      // the GPIO 1 pin low so it will come up in Codec mode.
      digitalWrite( vs1053Mode, LOW );
      // resetVS1053( false );

      // initialise the music player. We do this every time we switch between modes because
      // it performs some initialization required following the reset.
      if (! musicPlayer.begin()) {
        Serial.println(F("VS1053 not found on SPI"));
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
      //  Serial.println(F("DREQ pin is not an interrupt pin"));

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
        Serial.print(F("Stopping playback..."));
        musicPlayer.stopPlaying();
        wasPlaying = false;
        Serial.println(F("stopped"));
      }
    }

    void start( const char* filename ) {
      stop();
      if ( musicPlayer.startPlayingFile( filename ) )
      {
        Serial.print( F("Playing file: ") );
        Serial.println( filename );
        wasPlaying = true;
      }
      else
      {
        Serial.print( F("Failed to start playing file: ") );
        Serial.println( filename );
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
        Serial.println( F("Playback completed") );
        wasPlaying = false;
      }
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
        Serial.print( F("Error: Failed to play ") ); Serial.println( filename );
      }
    }
    else {
      Serial.print( F("Error: Track not found ") ); Serial.println( filename );
    }
  }
  else
    Serial.println( F("Error: Play track command while not in CODEC mode") );
}
void processCodecStatus( byte cmd, const byte* data )
{
  // Command 3 is the CODEC playback status [not yet functional]
  // Data is unused. Sends back 1-byte containing 0 if not playing, non-zero if playing. Always returns zero if not in CODEC mode.
  static byte counter = 1;
  ++counter;
  if ( counter >= 127 )
    counter = 1;
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
SoftwareSerial VS1053_MIDI( 0, synthPin );

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
  VS1053_MIDI.write( cmd );
  VS1053_MIDI.write( data1 );

  //Some commands only have one data byte. All cmds less than 0xBn have 2 data bytes
  //(sort of: http://253.ccarh.org/handout/midiprotocol/)
  if ( (cmd & 0xF0) <= 0xB0 || (cmd & 0xF0) == 0xE0)
    VS1053_MIDI.write( data2 );
}
void processMIDI( byte cmd, const byte* data )
{
  if( midiSynthMode ) {
    talkMIDI( cmd, data[0], data[1] );
  }
  else 
    Serial.println( F("Error: MIDI command while not in synthesizer mode") );
}
#endif // featureSynth

#if featureServos
// This section contains global variables to maintain the state of the two servos.
// These three defines control the range and the speed of both of the servos.
#define SERVO_MIN 0
#define SERVO_MAX 90
#define SERVO_STEP 1

// left_servo is the  servo object that controls the left servo/legs.
// left_move is a flag that stores the current 'moving' state of the left servo/legs.
// If true, the legs are continuously moved forward and then back. If false, then the
// servo/legs is set to the center of its normal travel (position of 45).
// left_pos holds the current position of the left servo/legs.
// left_step holds how much the position is changed with each iteration. The sign of
// the number indicates the direction. Currently either 1 or -1. It is changed when
// the position reaches the desired limits of 0 and 90 degrees.
struct servoData {
  Servo servo;
  volatile boolean move;
  int pos;
  int step;
} servos[2];
int servoDelay = 15;
int servoTimerId = -1;

void processServos()
{
  for( byte i = 0; i < 2; ++i ) {
    struct servoData& servo( servos[i] );
    if( servo.move ) {
      // If we are moving the servo/legs, then advance its current position
      // by the specified step...
      servo.pos += servo.step;
      // ...then change the direction of the step if the position has exceeded
      // the desired range.
      if( servo.pos >= SERVO_MAX ) {
        servo.step = -SERVO_STEP;
      }
      else if( servo.pos <= SERVO_MIN ) {
        servo.step = SERVO_STEP;
      }
    }
    else {
      // If we are not moving the left servo/legs, then set the position to the
      // center of the desired range.
      servo.pos = (SERVO_MAX + SERVO_MIN) / 2;
    }
    // tell servo to go to the newly calculated position in variable 'left_pos'
    servo.servo.write( servo.pos );
  }
}

void processServoMove( byte cmd, const byte* data )
{
  // Command 10 is the SERVO/Legs command. Data is interpreted as a bit field where
  // the least significant bit (0) sets the left side state, and next least significant
  // bit (1) sets the right side state. So every time you send a command, the state of both
  // sides is set. So the following values are valid:
  //  0 - turns off both sides.
  //  1 - turns the left side on and the right side off.
  //  2 - turns the left side off and the right side on.
  //  3 - turns on both sides.
  // Of course, you can define other protocols as well (and it won't hurt my feelings).
  servos[0].move = ( (data[0] & 1) != 0 );
  servos[1].move = ( (data[0] & 2) != 0 );
}
void processServoSpeed( byte cmd, const byte* data )
{
  // Command 11 is the SERVO speed command. The Data is interpreted as a speed where
  // zero (0) is the slowest (increments the servo counts once every 130 mSec) and
  // 127 is the fastest (increments the servo counts once every 2 mSec).
  servoDelay = (128 - data[0]);
  if ( servoDelay < 2 ) servoDelay = 2;
  else if ( servoDelay > 200 ) servoDelay = 200;
  timer.deleteTimer( servoTimerId );
  servoTimerId = timer.setInterval( servoDelay, processServos );
}
void processServoDirection( byte cmd, const byte* data )
{
  // Command 12 is the SERVO direction command. Data is interpreted as a bit field where
  // the least significant bit (0) set the direction for the left side, and the next least
  // significant bit (1) sets the direction for the right side. If the bit is zero, the servo
  // starts in the 'positive' direction, and if 1 in the negative direction. Of course, once
  // the servo reaches the end of its range, it reverses direction.
  servos[0].step = ( (data[0] & 1) != 0 ? -SERVO_STEP : SERVO_STEP );
  servos[1].step = ( (data[0] & 2) != 0 ? -SERVO_STEP : SERVO_STEP );
}
void processServoReset()
{
  for( byte i = 0; i < 2; ++i ) {
    struct servoData& servo( servos[i] );
    servo.move = false;
    servo.step = (i == 0 ? SERVO_STEP : -SERVO_STEP);
    servo.servo.write( servo.step );
  }
  servoDelay = 15;
  timer.deleteTimer( servoTimerId );
  servoTimerId = timer.setInterval( servoDelay, processServos );
}
#endif // featureServos

#if featureNeoPix
// The configuration will be set at runtime with information from the EV3.
struct neoPixelData {
  Adafruit_NeoPixel strip;
  uint32_t startColor;
  uint16_t startPosition;
  int16_t walkStep;
  int timerId;
  
  neoPixelData() : startColor( 0 ), startPosition( -1 ), walkStep( 0 ), timerId( -1 ) {}
} neoPixels[2];

void processNeoPix0()
{
  neoPixelData& npdata( neoPixels[0] );
  npdata.strip.rotatePixels( npdata.walkStep );
  npdata.strip.show();
}
void processNeoPix1()
{
  neoPixelData& npdata( neoPixels[1] );
  npdata.strip.rotatePixels( npdata.walkStep );
  npdata.strip.show();
}
void processNeoPixConfig( byte cmd, const byte* data )
{
  // Command 64 is NeoPixel Configure. Data0/1 is the number of pixels in the string attached to
  // pin 6, and Data2/3 is the number of pixels in the string attached to pin 7. Should be zero if
  // the string is not present.
  // uint16_t length = ((uint16_t) data[0] << 8 | data[1]);
  neoPixelData& npdata( neoPixels[data[0] & 0x1] );
  uint16_t length = data[1];
  byte brightness = data[2];
  if ( length > 0 ) {
    Serial.print( F("Configure NeoPixel strip ") ); Serial.print( data[0] & 0x1 ); 
        Serial.print( F(" length ") ); Serial.print(length); Serial.print( F(" brightness ") ); Serial.println(brightness);
    npdata.strip.updateType( NEO_GRB | NEO_KHZ800 );
    npdata.strip.updateLength( length );
    npdata.strip.setBrightness( brightness );
    npdata.strip.begin();
  }
  else {
    Serial.print( F("Disabling NeoPixel strip ") ); Serial.println( data[0] & 0x1 );
    npdata.strip.end();
  }
}
void processNeoPixShow( byte cmd, const byte* data )
{
  neoPixelData& npdata( neoPixels[data[0] & 0x1] );
  npdata.strip.show();
}
void processNeoPixSetStart( byte cmd, const byte* data )
{
  neoPixelData& npdata( neoPixels[data[0] & 0x1] );
  npdata.startPosition = data[1];
  npdata.startColor = Adafruit_NeoPixel::Color( data[2], data[3], data[4] );
}
void processNeoPixSetEnd( byte cmd, const byte* data )
{
  neoPixelData& npdata( neoPixels[data[0] & 0x1] );
  if( npdata.strip.numPixels() ) {
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
  neoPixelData& npdata( neoPixels[data[0] & 0x1] );
  if( npdata.strip.numPixels() ) {
    uint32_t color = Adafruit_NeoPixel::Color( data[2], data[3], data[4] );
    npdata.strip.setPixelRange( 0, color, npdata.strip.numPixels() - 1, color );
    npdata.strip.show();
  }
}
void processNeoPixRotate( byte cmd, const byte* data )
{
  neoPixelData& npdata( neoPixels[data[0] & 0x1] );
  if( npdata.strip.numPixels() ) {
    int16_t shift = (int8_t) data[1];
    npdata.strip.rotatePixels( shift );
    npdata.strip.show();
  }
}
void processNeoPixWalk( byte cmd, const byte* data )
{
  neoPixelData& npdata( neoPixels[data[0] & 0x1] );
  if( npdata.timerId >= 0 ) {
    timer.deleteTimer( npdata.timerId );
    npdata.timerId = -1;
  }
  npdata.walkStep = data[1];
  long interval = (long) data[2] << 4;
  if( npdata.walkStep > 0 && interval > 0 ) {
    Serial.print( F("Starting NeoPixel walk strip ") ); Serial.print(data[0] & 0x1); 
        Serial.print( F(" step ") ); Serial.print(npdata.walkStep); Serial.print( F(" interval ") ); Serial.println(interval);
    npdata.timerId = timer.setInterval( interval, (data[0] & 0x1 ? processNeoPix1 : processNeoPix0) );
  }
  else {
    Serial.print( F("Stopping NeoPixel walk strip ") ); Serial.println(data[0] & 0x1);
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

#if defined( vs1053Reset ) && defined( vs1053Mode )
// This function performs a reset of the VS1053 and brings it up in the specified
// mode. If 'midiSynth' is true, then it drives the GPIO 1 pin on the VS1053 high during
// the reset, which causes it to come up in MIDI Synthesizer mode. Otherwise, it drives
// GPIO 1 low during the reset, which brings it up in the Codec mode.
void resetVS1053( bool midiSynth )
{
  //Reset the VS1053
  pinMode( vs1053Mode, OUTPUT );
  digitalWrite( vs1053Mode, (midiSynth ? HIGH : LOW) ); // high brings it up in MIDI mode
  digitalWrite( vs1053Reset, LOW );
  delay(100);
  digitalWrite( vs1053Reset, HIGH );
  delay(100);
}
#endif // defined( vs1053Reset ) && defined( vs1053Mode )
void processMusicMode( byte cmd, const byte* data )
{
  // Command 1 is the Music mode.
  // Data is 0 for none, 1 for SYNTH mode, or 2 for CODEC mode.

  // Turn off any mode that is no longer desired.
#if featureSynth
  if( data[0] != 1 && midiSynthMode ) {
    Serial.println( F("Turning off MIDI SYNTH mode") );
    // Nothing to do...
    midiSynthMode = false;
  }
#endif // featureSynth
#if featureCodec
  if( data[0] != 2 && mp3CodecMode ) {
    Serial.println( F("Turning off CODEC mode") );
    VS1053_CODEC.exit();
    mp3CodecMode = false;
  }
#endif // featureCodec

  // Turn on any mode that is now requested.
  if( data[0] == 1 ) {
#if featureSynth
    if( !midiSynthMode ) {
      Serial.println( F("Turning on MIDI SYNTH mode") );
#if defined( vs1053Reset ) && defined( vs1053Mode )
      // Put into MIDI synthesizer mode.
      resetVS1053( true );
#endif // defined( vs1053Reset ) && defined( vs1053Mode )
      VS1053_MIDI.begin( 31250 ); // MIDI uses a 'strange baud rate'

      talkMIDI( MIDI_CHAN_MSG | 0, MIDI_CHAN_BANK, VS1053_BANK_MELODY );
      talkMIDI( MIDI_CHAN_PROGRAM | 0, VS1053_GM1_OCARINA, 0 );
      talkMIDI( MIDI_CHAN_MSG | 0, MIDI_CHAN_VOLUME, 127 );

      midiSynthMode = true;
    }
#else
    Serial.println( F("Error: MIDI commands not supported by this build") );
#endif // featureSynth
  }
  if( data[0] == 2 ) {
#if featureCodec
    if ( !mp3CodecMode ) {
      if ( usingAdaFruitMMS && sdCardPresent ) {
        Serial.println( F("Turning on CODEC mode") );
        VS1053_CODEC.enter();
        mp3CodecMode = true;
      }
      else if( !sdCardPresent )
        Serial.println( F("Error: SD card not present") );
      else
        Serial.println( F("Error: AdaFruit Music Maker Shield not present") );
    }
#else
    Serial.println( F("Error: Codec commands not supported by this build") );
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
    VS1053_CODEC.setVolume( volume );
  }
#endif // featureCodec
}
void processReset( byte cmd, const byte* data )
{
  // Command 127 is the 'restore default state' command. It basically sets everything back to
  // the power on defaults so that the EV3 program is starting from a known state.
  Serial.println( F("Restoring default state") );
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
void processUnrecognized( byte cmd, const byte* data )
{
  Serial.print( F("Error: Unrecognized command ") ); Serial.println(cmd);
}

void showCapabilities()
{
  Serial.begin(115200);
  Serial.println( F("EV3 Arduino Extensions - " __DATE__ " " __TIME__) );

#if featureCodec
  Serial.println( F("Codec support for AdaFruit Music Maker Shield.") );
  Serial.println( F("- supports .mp3 and .mid files.") );
  if( usingAdaFruitMMS ) {
    Serial.println( F("- detected AdaFruit Music Maker Shield") );
    if ( sdCardPresent )
      Serial.println( F("- detected SD card") );
  }
#endif // featureCodec

#if featureSynth
  Serial.print( F("MIDI synthesizer support on pin ") ); Serial.println( synthPin );
  Serial.println( F("- supports AdaFruit Music Maker Shield OR SparkFun Music Instrument Shield") );
#endif // featureSynth

#if defined( vs1053Mode ) && defined( vs1053Reset )
  Serial.print( F("Switching music modes requires RST on pin ") ); Serial.print( vs1053Reset ); Serial.print( F(" and GPIO1 to pin ") ); Serial.println( vs1053Mode );
#endif // defined( vs1053Mode ) && defined( vs1053Reset )

#if featureServos
  Serial.print( F("Servo support for servos on pin ") ); Serial.print( servo0Pin ); Serial.print( F( " and pin ") ); Serial.println( servo1Pin );
#endif // featureServos

#if featureNeoPix
  Serial.print( F("NeoPixel support for strips on pin ") ); Serial.print( neoPix0Pin ); Serial.print( F(" and pin ") ); Serial.println( neoPix1Pin );
#endif // featureNeoPix
}

void setup()
{
  Wire.begin( ardAdd );
  Wire.onReceive( receiveData );
  Wire.onRequest( requestData );

#if defined( vs1053Mode ) && defined( vs1053Reset )
  // These pins are used to control the VS1053 on the AdaFruit shield, so must be output.
  pinMode( vs1053Mode, OUTPUT );
  pinMode( vs1053Reset, OUTPUT );
#endif

#if featureCodec
  // If we can enter CODEC mode, then we interpret that to mean that we are running with
  // the AdaFruit Music Maker Shield.
  usingAdaFruitMMS = VS1053_CODEC.enter();
  VS1053_CODEC.exit();
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
  servos[0].servo.attach( servo0Pin );  // attaches the servo object to the appropriate pin.
  servos[1].servo.attach( servo1Pin );
  processServoReset();
#endif // featureServos

#if featureNeoPix
  neoPixels[0].strip.setPin( neoPix0Pin );
  neoPixels[1].strip.setPin( neoPix1Pin );
  processNeoPixReset();
#endif // featureNeoPix

  showCapabilities();
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

// A simple queue for commands received from the EV3.
// cmdQueueWrite is the next index at which a byte will be written.
// cmdQueueRead is the next index at which a byte will be read.
// So cmdQueueRead == cmdQueueWrite means the queue is empty.
// and cmdQueueRead == (cmdQueueWrite + 1) % sizeof(cmdQueue) means the queue is full.
struct xferQueue {
  byte queue[16];
  byte writeIdx;
  byte readIdx;
  bool overflowOccured;
  xferQueue() : writeIdx( 0 ), readIdx( 0 ), overflowOccured( false ) {}
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
  inline bool overflow() {
    bool data = overflowOccured;
    overflowOccured = false;
    return( data );
  }
} cmdQueue, reqQueue;
volatile bool cmdProcessing = false;

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
  if( !reqQueue.empty() ) {
    Wire.write( reqQueue.pop_front() );
  }
  else {  
    byte data = 0x80;
    if( !cmdQueue.empty() || cmdProcessing )
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
    Wire.write( data );
  }
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
  { 3, 1, processCodecStatus },
  { 5, 1, processCodecStop },
#endif // featureCodec
#if featureServos
  { 10, 1, processServoMove },
  { 11, 1, processServoSpeed },
  { 12, 1, processServoDirection },
#endif // featureServos
#if featureNeoPix
  { 20, 3, processNeoPixConfig },
  { 21, 1, processNeoPixShow },
  { 24, 5, processNeoPixSetStart },
  { 25, 5, processNeoPixSetEnd },
  { 26, 5, processNeoPixSetAll },
  { 30, 3, processNeoPixRotate },
  { 31, 3, processNeoPixWalk },
#endif // featureNeoPix
  { 127, 0, processReset },
};

void processCmdQueue()
{
  // Serial.print("write: "); Serial.print(cmdQueueWrite); Serial.print(", read: "); Serial.println(cmdQueueRead);

  if ( cmdQueue.overflow() )
    Serial.println( F("Error: Command queue overflow, likely command errors") );

  while( !cmdQueue.empty() ) {
    byte cmd = cmdQueue.peek_front();
    commandFunction cmdFunc = processUnrecognized;
    byte dataBytes = 0;
#if featureSynth
    // handle all of the MIDI command separately because they cover a large range of values 
    // (everything with the msb set) and can have 1 or 2 data bytes.
    if( (cmd & 0x80) != 0 ) {
      cmdFunc = processMIDI;
      dataBytes = 1;
      if ( (cmd & 0xF0) != 0xC0 && (cmd & 0xF0) != 0xD0 )
        dataBytes = 2;
    }
    else {
#endif // featureSynth
      for ( int i = 0; i < sizeof(commandToFuncMap) / sizeof(commandToFuncMap[0]); ++i )
      {
        byte mapCmd = pgm_read_byte_near( &commandToFuncMap[i].cmd );
        // Serial.print( "i " ); Serial.print( i ); Serial.print( " cmd "); Serial.println( mapCmd );
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
      byte data[5];
      for ( byte i = 0; i < dataBytes; ++i )
        data[i] = cmdQueue.pop_front();
      for ( byte i = dataBytes; i < sizeof(data); ++i )
        data[i] = 0;

      // now that we have the data bytes collected, invoke the processing function.
      (*cmdFunc)( cmd, data );
  
      cmdProcessing = false;
    }
    else
    {
      // At the moment, I cannot send six bytes at once from the EV3. So I tollerate getting them
      // as 2+4.
      if( cmdQueue.bytes() == 2 && dataBytes == 5 )
        break;
      // data byte has not arrived yet. Process it next time. Why don't we wait 'forever' for it to 
      // arrive? Because sometimes we get garbage over the wire, and if it looks like a command then
      // it gets stuck. This way, if the rest of the command does not show up fairly quickly, then
      // we discard it.
      Serial.println( cmdQueue.bytes() );
      static byte s_tries = 0;
      ++s_tries;
      if ( s_tries > 3 )
      {
        Serial.print( F("Error: Incomplete command ") ); Serial.println(cmd);
        cmdQueue.pop_front();
        s_tries = 0;
      }
      break; // out of the while loop
    }
  } //   while( !cmdQueue.empty() )
}
