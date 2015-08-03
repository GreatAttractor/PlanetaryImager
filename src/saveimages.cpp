/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2015  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "saveimages.h"
#include <QFile>
#include <QThread>
#include <QDebug>
#include <functional>
#include <QQueue>
#include "utils.h"

#include <QMutex>
#include <QMutexLocker>
#include <QtConcurrent/QtConcurrent>
#include <functional>

using namespace std;
using namespace std::placeholders;

class FileWriter : public ImageHandler {
public:
  virtual void handle(const ImageDataPtr& imageData) = 0;
};
typedef shared_ptr<FileWriter> FileWriterPtr;
typedef function<FileWriterPtr(const QString &filename, bool buffered)> FileWriterFactory;


struct __attribute__ ((__packed__)) SER_Header {
    char fileId[14] = {'L', 'U', 'C', 'A', 'M', '-', 'R','E','C','O','R','D','E','R'};
    int32_t luId = 0;
    enum ColorId {
        MONO = 0,
        BAYER_RGGB = 8,
        BAYER_GRBG = 9,
        BAYER_GBRG = 10,
        BAYER_BGGR = 11,
        BAYER_CYYM = 16,
        BAYER_YCMY = 17,
        BAYER_YMCY = 18,
        BAYER_MYYC = 19,
        RGB = 100,
        BGR = 101,
    };
    int32_t colorId = MONO;
    enum Endian { BigEndian = 0, LittleEndian = 1 };
    int32_t endian = BigEndian;
    uint32_t imageWidth = 0;
    uint32_t imageHeight = 0;
    uint32_t pixelDepth = 0;
    uint32_t frames = 0;
    char observer[40] = {};
    char camera[40] = {};
    char telescope[40] = {};
    uint8_t datetime[8] = {};
    uint8_t datetime_utc[8] = {};
};

class SER_Writer : public FileWriter {
public:
  SER_Writer(const QString &filename, bool buffered);
  ~SER_Writer();
  virtual void handle(const ImageDataPtr& imageData);
private:
  QFile file;
  SER_Header header;
  uint32_t frames;
};



SER_Writer::SER_Writer(const QString& filename, bool buffered) : file("%1.ser"_q % filename)
{
  if(buffered)
    file.open(QIODevice::ReadWrite);
  else
    file.open(QIODevice::ReadWrite | QIODevice::Unbuffered);
  
  file.write(reinterpret_cast<char*>(&header), sizeof(header));
  file.flush();

}

SER_Writer::~SER_Writer()
{
  SER_Header *mem_header = reinterpret_cast<SER_Header*>(file.map(0, sizeof(SER_Header)));
  if(!mem_header) {
    qDebug() << file.errorString();
  }
  mem_header->frames = frames;
  file.close();
}


void SER_Writer::handle(const ImageDataPtr& imageData)
{
  if(! header.imageWidth) {
    header.colorId = imageData->channels() == 1 ? SER_Header::MONO : SER_Header::RGB;
    header.pixelDepth = imageData->bpp();
    header.imageWidth = imageData->width();
    header.imageHeight = imageData->height();
  }
  file.write(reinterpret_cast<const char*>(imageData->data()), imageData->size());
  frames++;
}


class WriterThreadWorker;
class SaveImages::Private {
public:
    Private(SaveImages *q);
    QThread recordingThread;
    QString filename;
    Format format = SER;
    FileWriterFactory writerFactory();
    WriterThreadWorker *worker = nullptr;
    bool buffered;
private:
    SaveImages *q;
};

SaveImages::Private::Private(SaveImages* q) : q {q}
{
}

SaveImages::SaveImages(QObject* parent) : QObject(parent), dpointer(this)
{
}


SaveImages::~SaveImages()
{
}


void SaveImages::setOutput(const QString& filename, Format format)
{
  d->filename = filename;
  d->format = format;
}

FileWriterFactory SaveImages::Private::writerFactory()
{
  if(filename.isEmpty()) {
    return {};
  }
  static map<Format, FileWriterFactory> factories {
    {SER, [](const QString &filename, bool buffered){ return make_shared<SER_Writer>(filename, buffered); }},
  };
  
  return factories[format];
}

class WriterThreadWorker : public QObject {
  Q_OBJECT
public:
  explicit WriterThreadWorker ( const function<FileWriterPtr()> &fileWriterFactory, QObject* parent = 0 );
public slots:
  virtual void handle(const ImageDataPtr& imageData);
  void finish() {stop = true; }
  void run();
signals:
  void saveFPS(double fps);
  void savedFrames(uint64_t frames);
private:
  function<FileWriterPtr()> fileWriterFactory;
  QQueue<ImageDataPtr> frames;
  bool stop = false;
  QMutex mutex;
};

WriterThreadWorker::WriterThreadWorker ( const function< FileWriterPtr()>& fileWriterFactory, QObject* parent )
  : QObject(parent), fileWriterFactory(fileWriterFactory)
{
}


void WriterThreadWorker::handle(const ImageDataPtr& imageData)
{
  QMutexLocker lock(&mutex);
  frames.enqueue(imageData); 
}


void WriterThreadWorker::run()
{
  auto fileWriter = fileWriterFactory();
  fps savefps{[=](double fps){ emit saveFPS(fps);}};
  uint64_t frames = 0;
  while(!stop) {
    if(this->frames.size()>0) {
      {
	QMutexLocker lock(&mutex);
	fileWriter->handle(this->frames.dequeue());
      }
      savefps.add_frame();
      emit savedFrames(++frames);
    }
  }
}

void SaveImages::setBuffered(bool buffered)
{
  d->buffered = buffered;
}


void SaveImages::handle(const ImageDataPtr& imageData)
{
  if(!d->worker)
    return;
  QtConcurrent::run(bind(&WriterThreadWorker::handle, d->worker, imageData));
}

void SaveImages::startRecording()
{
  auto writerFactory = d->writerFactory();
  if(writerFactory) {
    d->worker = new WriterThreadWorker(bind(writerFactory, d->filename, d->buffered));
    connect(d->worker, &WriterThreadWorker::savedFrames, bind(&SaveImages::savedFrames, this, _1));
    connect(d->worker, SIGNAL(saveFPS(double)), this, SIGNAL(saveFPS(double)));
    d->worker->moveToThread(&d->recordingThread);
    connect(&d->recordingThread, SIGNAL(started()), d->worker, SLOT(run()));
    connect(&d->recordingThread, SIGNAL(finished()), d->worker, SLOT(deleteLater()));
    d->recordingThread.start();
  }
}


void SaveImages::endRecording()
{
  d->worker->finish();
  d->recordingThread.quit();
  QtConcurrent::run([=] {
    d->recordingThread.wait();
    d->recordingThread.terminate();
    d->worker = nullptr;
  });
}


#include "saveimages.moc"
