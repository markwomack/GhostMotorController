//
// Licensed under the MIT license.
// See accompanying LICENSE file for details.
//

#ifndef REMOTESERIALREADERTASK_H
#define REMOTESERIALREADERTASK_H

#include <DebugMsgs.h>
#include <CascadePrinter.h>
#include <Task.h>

class RemoteSerialReaderTask : public Task {
  public:
    void setSrcSerial(HardwareSerial* srcSerial) {
      _srcSerial = srcSerial;
    };

    void setDstPrinter(CascadePrinter* dstCascadePrinter) {
      _dstCascadePrinter = dstCascadePrinter;
    }

    void update(void) {
      if (_srcSerial->available()) {
        uint32_t size = _srcSerial->readBytes(buffer, sizeof(buffer));
        if (size == 0) {
          return;
        }

        //DebugMsgs.debug().printfln("Starting transfer for %d bytes", size);
        uint8_t lineBuffer[4096];
        for (size_t totalSize = 0; totalSize < size;) {
          bool readLF = false;
          size_t sendSize = readUntilLF(buffer+totalSize, size - totalSize, lineBuffer, sizeof(lineBuffer), &readLF);
          //DebugMsgs.debug().printfln("totalSize %d, sendSize %d, lineBuffer '%s'", totalSize, sendSize, lineBuffer);
          totalSize += sendSize;
          if (readLF) {
            _dstCascadePrinter->println((char*)lineBuffer);
          } else {
            _dstCascadePrinter->print((char*)lineBuffer);
            _dstCascadePrinter->flush();
          }
        }
        //DebugMsgs.debug().println("Transfer complete");
      }
    };

  private:
    HardwareSerial* _srcSerial;
    CascadePrinter* _dstCascadePrinter;
    uint8_t buffer[16384];

    size_t readUntilLF(uint8_t* src, size_t sizeSrc, uint8_t* dst, size_t sizeDst, bool* readLF) {
      size_t x = 0;
      *readLF = false;
      for (x = 0; x < sizeSrc && x < sizeDst-1; x++) {
        if (src[x] == '\n') {
          *readLF = true;
          break;
        } else {
          dst[x] = src[x];
        }
      }

      dst[x] = 0;
      return x + (*readLF ? 1 : 0);
    };
};

#endif // REMOTESERIALREADERTASK_H
