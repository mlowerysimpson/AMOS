/*
port of Peter Jansen's Arduino driver for Benewake TFMini time-of-flight distance sensor to Raspberry Pi
by Murray Lowery-Simpson (August 18, 2018)
This code is open source software in the public domain.

THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

The names of the contributors may not be used to endorse or promote products
derived from this software without specific prior written permission.
*/
#include <wiringPi.h>
#include <wiringSerial.h>

// Defines
#define TFMINI_BAUDRATE   115200
#define TFMINI_DEBUGMODE  1

// The frame size is nominally 9 characters, but we don't include the first two 0x59's marking the start of the frame
#define TFMINI_FRAME_SIZE                 7

// Timeouts
#define TFMINI_MAXBYTESBEFOREHEADER       30
#define TFMINI_MAX_MEASUREMENT_ATTEMPTS   10

// States
#define READY                             0
#define ERROR_SERIAL_NOHEADER             1
#define ERROR_SERIAL_BADCHECKSUM          2
#define ERROR_SERIAL_TOOMANYTRIES         3
#define MEASUREMENT_OK                    10

//minimum acceptable signal strength
#define MIN_SIGNAL_STRENGTH 20

//
// Driver Class Definition
//
class TFMini {
  public: 
    TFMini(void);//constructor
    ~TFMini();//destructor

    // Configuration
    bool begin(int fd);
    void setSingleScanMode();
    
    // Data collection
    unsigned short getDistance();
    unsigned short getRecentSignalStrength();
    void externalTrigger();

  private:
    int m_fd;//file descriptor for the serial port (returned by the wiringPi's serialOpen function)
    int state;
    unsigned short distance;
    unsigned short strength;
    
    // Low-level communication
    bool setStandardOutputMode();//set to use the standard output mode
    bool setConfigMode();//go into configuration mode
	bool exitConfigMode();//exit the configuration mode
	int takeMeasurement();
    
};

