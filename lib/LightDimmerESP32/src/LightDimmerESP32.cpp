/*
 * Light Dimmer Library for Arduino
 *
 * Copyright Jean-Luc Béchennec 2018
 *
 * This software is distributed under the GNU Public Licence v2 (GPLv2)
 *
 * Please read the LICENCE file
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <LightDimmerESP32.h>

LightDimmerESP32 *LightDimmerESP32::sLightList = NULL;
LightDimmerESP32 *LightDimmerESP32::sCurrent = NULL;

LightDimmerESP32::LightDimmerESP32()
  : mState(LD_OFF),
    mMax(1023u),
    mRiseTime(250u),
    mFallTime(250u),
    mOnTime(200u),
    mPeriod(900u),
    mNextEventDate(0u),
    mValue(0u),
    old_mValue(0u),
    mPin(63),
    mBlink(false),
    mOff(LOW)
{
  mNext = sLightList;
  sLightList = this;
}

void LightDimmerESP32::begin(const uint8_t inPin, const uint8_t inOn)
{
  mPin = inPin;
  mOff = !inOn;
  pinMode(mPin, OUTPUT);
  /* At start, le LED is switched off */
  digitalWrite(mPin, mOff);
}

void LightDimmerESP32::setupMax(const uint16_t inMax)
{
  mNextEventDate = millis() + mRiseTime;
  mState = LD_RISING;
  mBlink = false;
  old_mValue=mValue; 
  mMax = inMax;
}

void LightDimmerESP32::on()
{
  switch (mState)
  {
    case LD_OFF:
      mNextEventDate = millis() + mRiseTime;
      mState = LD_RISING;
      break;
    case LD_FALLING:
      uint32_t remainingTime = mNextEventDate - millis();
      mNextEventDate = millis() + (uint32_t)mRiseTime * remainingTime / (uint32_t)mFallTime;
      mState = LD_RISING;
      break;
  }
  mBlink = false;
}

void LightDimmerESP32::off()
{
  switch (mState)
  {
    case LD_ON:
      mNextEventDate = millis() + mFallTime;
      mState = LD_FALLING;
      break;
    case LD_RISING:
      uint32_t remainingTime = mNextEventDate - millis();
      mNextEventDate = millis() + (uint32_t)mFallTime * remainingTime / (uint32_t)mRiseTime;
      mState = LD_FALLING;
      break;
  }
  mBlink = false;
}

void LightDimmerESP32::startBlink()
{
  if (mPeriod >= mRiseTime + mOnTime + mFallTime) {
    mBlink = true;
  }
}

/*
 * LightDimmerESP32::updateState implements the state machine associated to each
 * LED.
 */
void LightDimmerESP32::updateState()
{
  uint32_t currentDate = millis();
  switch (mState) {
    case LD_OFF:
      if (mBlink) {
        if (currentDate >= mNextEventDate) {
          mNextEventDate = millis() + mRiseTime;
          mState = LD_RISING; 
        }
      }
      break;
    case LD_RISING:
      if (currentDate < mNextEventDate) {
        uint16_t dzielnik = 1023 * (currentDate - (mNextEventDate - mRiseTime)) / mRiseTime;
        mValue=old_mValue + dzielnik*(mMax - old_mValue)/1023;
        //Serial.print("old_mValue"); Serial.print("="); Serial.print(old_mValue); Serial.print(" : ");
        //Serial.print("dzielnik"); Serial.print("="); Serial.print(dzielnik); Serial.print(" : ");
      }
      else {
        mValue = mOff ? 1023 - mMax : mMax ;
        mNextEventDate = currentDate + mPeriod - mOnTime - mRiseTime -mFallTime;
        mState = LD_ON;
      }
      break;
    case LD_ON:
      if (mBlink) {
        if (currentDate >= mNextEventDate) {
          mNextEventDate = currentDate + mFallTime;
          mState = LD_FALLING;
        }
      }
      break;
    case LD_FALLING:
      if (currentDate < mNextEventDate) {
        uint16_t value = mMax * (currentDate - (mNextEventDate - mFallTime)) / mFallTime;
        mValue = mOff ? value + 1023 - mMax : mMax - value;
      }
      else {
        mValue = mOff ? 1023 : 0;
        mState = LD_OFF;
        mNextEventDate = currentDate + mPeriod - mOnTime - mRiseTime - mFallTime;
      }
      break;
  }
  updateOutput();
}

void LightDimmerESP32::updateOutput()
{
  //long lmValue=(2*(mValue)*(mValue)+mValue)/2000; //funkcja kwadratowa nie sprawdziła się, może sinus ?
  //analogWrite(mPin, mValue);
  //ledcWrite(0, mValue);//ledChannel, dutyCycle
  ledcWrite(mPin, mValue);
  //Serial.print(mState); Serial.print(" mValue:"); Serial.println(lmValue); delay(100);

}

/*
 * LightDimmerESP32::update shall be called in loop to allow the library to update
 * the state of each LED. If the call frequency is not high enough you will
 * get discontinuous update in the fading and brightening process for
 * LightDimmerESP32 objects (those using hardware PWM). LightDimmerESP32Soft object
 * (those using software PWM) will flicker. So you shall never use the delay
 * function in your sketch.
 */
void LightDimmerESP32::update()
{
  LightDimmerESP32 *ld = sLightList;
  while (ld != NULL) {
    ld->updateState();
    ld = ld->mNext;
  }
}

/*
 * As an option, update can take as argument the number of objects to update.
 * This allows you to more finely interlace LightDimmerESP32's work and the rest
 * of your sketch. Passing a number of objects greater than the number of
 * objects declared has no particular effect other than updating several times
 * and unnecessarily the same object.
 */
void LightDimmerESP32::update(const uint8_t inHowMany)
{
  for (uint8_t i = 0; i < inHowMany; i++) {
    if (sCurrent == NULL) sCurrent = sLightList;
    sCurrent->updateState();
    sCurrent = sCurrent->mNext;
  }
}

/*
 * LightDimmerESP32Soft::updateOutput compute the state of the output according to
 * the value of the PWM
 */
void LightDimmerESP32Soft::updateOutput()
{
  mDuty += mOff + (mValue >> 3);       /* Add the value to the duty on 5 bits */
  uint8_t out = ((mDuty & 0xE0) != 0); /* The output is the carry             */
  digitalWrite(mPin, out);             /* Set the OUTPUT                      */
  mDuty &= 0x1F;                       /* Clear the carry                     */
}
