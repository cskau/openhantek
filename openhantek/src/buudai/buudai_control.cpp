////////////////////////////////////////////////////////////////////////////////
//
//  OpenBuudai
//  buudai/control.cpp
//
//  Copyright (C) 2008, 2009  Oleg Khudyakov
//  prcoder@potrebitel.ru
//  Copyright (C) 2010  Oliver Haag
//  oliver.haag@gmail.com
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along with
//  this program.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////


#include <QList>
#include <QMutex>
#include <QDebug>


#include "buudai/buudai_control.h"

#include "helper.h"
#include "buudai/buudai_device.h"
#include "buudai/buudai_types.h"


namespace Buudai {
	/// \brief Initializes the command buffers and lists.
	/// \param parent The parent widget.
	Control::Control(QObject *parent) : DsoControl(parent) {
		// Values for the Gain and Timebase enums
		gainSteps << 0.05 << 0.10 << 0.20 << 0.50 << 1.00 << 2.00 << 5.00;
		samplerateChannelMax = 48e6;
		samplerateFastMax = 48e6;
		samplerateMax = samplerateChannelMax;
		samplerateDivider = 1;
		triggerPosition = 0;
		triggerSlope = Dso::SLOPE_POSITIVE;
		triggerSpecial = false;
		triggerSource = 0;
		triggerPoint = 0;

		// Channel level data
		for (unsigned int channel = 0; channel < BUUDAI_CHANNELS; channel++) {
			for (unsigned int gainId = 0; gainId < GAIN_COUNT; gainId++) {
				channelLevels[channel][gainId]= 0x0000;
			}
			sampleRange[channel] = 255;
			offsetReal[channel] = double(135)/sampleRange[channel];
			gain[channel] = GAIN_100MV;
		}
		
		// USB device
		device = new Device(this);
		
		// Sample buffers
		for (unsigned int channel = 0; channel < BUUDAI_CHANNELS; channel++) {
			samples.append(0);
			samplesSize.append(0);
		}
		
		connect(device, SIGNAL(disconnected()), this, SLOT(disconnectDevice()));
	}
	
	/// \brief Disconnects the device.
	Control::~Control() {
		device->disconnect();
	}
	
	/// \brief Gets the physical channel count for this oscilloscope.
	/// \returns The number of physical channels.
	unsigned int Control::getChannelCount() {
		return BUUDAI_CHANNELS;
	}
	
	/// \brief Handles all USB things until the device gets disconnected.
	void Control::run() {
		int errorCode, cycleCounter = 0;

		while (!terminate) {
			errorCode = getSamples(true);
			if (errorCode < 0)
				qDebug("Getting sample data failed: %s", Helper::libUsbErrorString(errorCode).toLocal8Bit().data());

		}
		
		device->disconnect();
		emit statusMessage(tr("The device has been disconnected"), 0);
	}
	
	/// \brief Calculates the trigger point from the CommandGetCaptureState data.
	/// \param value The data value that contains the trigger point.
	/// \return The calculated trigger point for the given data.
	unsigned short int Control::calculateTriggerPoint(unsigned short int value) {
		unsigned short int result = value;
    
		// Each set bit inverts all bits with a lower value
		for (unsigned short int bitValue = 1; bitValue; bitValue <<= 1)
			if (result & bitValue)
				result ^= bitValue - 1;
		
		return result;
	}
	
	
	/// \brief Gets sample data from the oscilloscope and converts it.
	/// \return 0 on success, libusb error code on error.
	int Control::getSamples(bool process) {
		int errorCode;

		// Save raw data to temporary buffer
		unsigned int dataCount = bufferSize * BUUDAI_CHANNELS;
		unsigned int dataLength = dataCount;
		
		unsigned char data[dataLength];
		errorCode = device->bulkReadMulti(data, dataLength);
		if (errorCode < 0)
			return errorCode;

		

		// Process the data only if we want it
		if (process) {
			// How much data did we really receive?
			dataLength = errorCode;
			dataCount = dataLength;
			
			samplesMutex.lock();
			
			unsigned int channelDataCount = dataCount / BUUDAI_CHANNELS;

			for (int channel = 0; channel < BUUDAI_CHANNELS; channel++) {
				// Reallocate memory for samples if the sample count has changed
				if (!samples[channel] || samplesSize[channel] != channelDataCount) {
					if (samples[channel])
						delete samples[channel];
					samples[channel] = new double[channelDataCount];
					samplesSize[channel] = channelDataCount;
				}

				// Convert data from the oscilloscope and write it into the sample buffer
				unsigned int bufferPosition = triggerPoint * 2;
				for (unsigned int realPosition = 0; realPosition < channelDataCount; realPosition++, bufferPosition += 2) {
					if (bufferPosition >= dataCount)
						bufferPosition %= dataCount;
								
					samples[channel][realPosition] = ((double) data[bufferPosition + channel] / sampleRange[channel] - offsetReal[channel]) * gainSteps[gain[channel]];
				}
			}
			
			samplesMutex.unlock();
			emit samplesAvailable(&(samples), &(samplesSize), (double) samplerateMax / samplerateDivider, &(samplesMutex));
		} // if (process)

		return 0;
	}
	
	/// \brief Sets the size of the sample buffer without updating dependencies.
	/// \param size The buffer size that should be met (S).
	/// \return The buffer size that has been set.
	unsigned long int Control::updateBufferSize(unsigned long int size) {
		BufferSizeId sizeId = (size <= BUFFER_SMALL) ? BUFFERID_SMALL : BUFFERID_LARGE;
		bufferSize = (sizeId == BUFFERID_SMALL) ? BUFFER_SMALL : BUFFER_LARGE;
		return bufferSize;
	}
	
	/// \brief Try to connect to the oscilloscope.
	void Control::connectDevice() {
		int errorCode;
		
		emit statusMessage(device->search(), 0);
		if (!device->isConnected())
			return;
		
		bool unsupported = true;
		switch(device->getModel()) {
			case MODEL_DDS120:
				unsupported = false;
				break;
			case MODEL_DDS140:
				break;
			
			default:
				device->disconnect();
				emit statusMessage(tr("Unknown model"), 0);
				return;
		}
		
		if (unsupported)
			qDebug("Warning: This Buudai DSO model isn't supported officially yet, so it may not be working as expected. Reports about your experiences are very welcome though (Please open a feature request in the tracker at https://sf.net/projects/openbuudai/ or email me directly to oliver.haag@gmail.com). If it's working perfectly I can remove this warning, if not it should be possible to get it working with your help soon.");

		// Maximum possible samplerate for a single channel
		switch(device->getModel()) {
			case MODEL_DDS120:
				samplerateChannelMax = 48e6;
				samplerateFastMax = 48e6;
				break;
			
			case MODEL_DDS140:
				samplerateChannelMax = 48e6;
				samplerateFastMax = 100e6;
				break;
			default: // just to quite the compiler
				return;
		}
		samplerateMax = samplerateChannelMax;
		samplerateDivider = 1;

		DsoControl::connectDevice();
	}

	bool Control::deviceFound() {
		QString searchResult = device->search();
		qDebug() << "search result: " << searchResult;
		return searchResult.startsWith(tr("Device found:")); // not very robust...
	}

	/// \brief Sets the size of the oscilloscopes sample buffer.
	/// \param size The buffer size that should be met (S).
	/// \return The buffer size that has been set.
	unsigned long int Control::setBufferSize(unsigned long int size) {
		if (!device->isConnected())
			return 0;
		
		qDebug() << "setBufferSize: " << size;
		updateBufferSize(size);
		
		setTriggerPosition(triggerPosition);
		setSampleRate(samplerateMax / samplerateDivider);
		setTriggerSlope(triggerSlope);
		
		return bufferSize;
	}
	
	/// \brief Sets the sample rate of the oscilloscope.
	/// \param sampleRate The sample rate that should be met (S/s).
	/// \return The sample rate that has been set.
	unsigned long int Control::setSampleRate(unsigned long int sampleRate) {
		if (!device->isConnected() || sampleRate == 0)
			return 0;
		qDebug() << "setSampleRate: " << sampleRate;
		return sampleRate;
	}	
	
	/// \brief Enables/disables filtering of the given channel.
	/// \param channel The channel that should be set.
	/// \param used true if the channel should be sampled.
	/// \return 0 on success, -1 on invalid channel.
	int Control::setChannelUsed(unsigned int channel, bool used) {
		if (!device->isConnected())
			return -2;
		
		if (channel >= BUUDAI_CHANNELS)
			return -1;
		
		return 0;
	}
	
	/// \brief Set the coupling for the given channel.
	/// \param channel The channel that should be set.
	/// \param coupling The new coupling for the channel.
	/// \return 0 on success, -1 on invalid channel.
	int Control::setCoupling(unsigned int channel, Dso::Coupling coupling) {
		if (!device->isConnected())
			return -2;
		
		if (channel >= BUUDAI_CHANNELS)
			return -1;
		
		return 0;
	}
	
	/// \brief Sets the gain for the given channel.
	/// \param channel The channel that should be set.
	/// \param gain The gain that should be met (V/div).
	/// \return The gain that has been set, -1 on invalid channel.
	double Control::setGain(unsigned int channel, double gain) {
		if (!device->isConnected())
			return -2;
		
		if (channel >= BUUDAI_CHANNELS)
			return -1;
		
		// Find lowest gain voltage thats at least as high as the requested
		int gainId;
		for (gainId = 0; gainId < GAIN_COUNT - 1; gainId++)
			if (gainSteps[gainId] >= gain)
				break;
		
		return gainSteps[gainId];
	}
	
	/// \brief Set the offset for the given channel.
	/// \param channel The channel that should be set.
	/// \param offset The new offset value (0.0 - 1.0).
	/// \return The offset that has been set, -1.0 on invalid channel.
	double Control::setOffset(unsigned int channel, double offset) {
		if (!device->isConnected())
			return -2;
		
		if (channel >= BUUDAI_CHANNELS)
			return -1;
		
		return 0.0;
	}
	
	/// \brief Set the trigger mode.
	/// \return 0 on success, -1 on invalid mode.
	int Control::setTriggerMode(Dso::TriggerMode mode) {
		if (!device->isConnected())
			return -2;
		
		if (mode < Dso::TRIGGERMODE_AUTO || mode > Dso::TRIGGERMODE_SINGLE)
			return -1;
		
		triggerMode = mode;
		return 0;
	}
	
	/// \brief Set the trigger source.
	/// \param special true for a special channel (EXT, ...) as trigger source.
	/// \param id The number of the channel, that should be used as trigger.
	/// \return 0 on success, -1 on invalid channel.
	int Control::setTriggerSource(bool special, unsigned int id) {
		if (!device->isConnected())
			return -2;
		
		if ((!special && id >= BUUDAI_CHANNELS) || (special && id >= BUUDAI_SPECIAL_CHANNELS))
			return -1;
		
		// Generate trigger source value that will be transmitted
		int sourceValue;
		sourceValue = TRIGGER_CH1 - id;
		
		triggerSpecial = special;
		triggerSource = id;
		
		setTriggerLevel(id, triggerLevel[id]);
		
		return 0;
	}
	
	/// \brief Set the trigger level.
	/// \param channel The channel that should be set.
	/// \param level The new trigger level (V).
	/// \return The trigger level that has been set, -1.0 on invalid channel.
	double Control::setTriggerLevel(unsigned int channel, double level) {
		if (!device->isConnected())
			return -2;
		
		if (channel >= BUUDAI_CHANNELS)
			return -1.0;
		
		return (double) level;
	}
	
	/// \brief Set the trigger slope.
	/// \param slope The Slope that should cause a trigger.
	/// \return 0 on success, -1 on invalid slope.
	int Control::setTriggerSlope(Dso::Slope slope) {
		if (!device->isConnected())
			return -2;
		
		if (slope != Dso::SLOPE_NEGATIVE && slope != Dso::SLOPE_POSITIVE)
			return -1;
		
		triggerSlope = slope;
		return 0;
	}
	
	/// \brief Set the trigger position.
	/// \param position The new trigger position (in s).
	/// \return The trigger position that has been set.
	double Control::setTriggerPosition(double position) {
		if (!device->isConnected())
			return -2;
		
		// All trigger positions are measured in samples
		unsigned long int positionSamples = position * samplerateMax / samplerateDivider;
		
		triggerPosition = position;
		return (double) positionSamples / samplerateMax * samplerateDivider;
	}

#ifdef DEBUG
	/// \brief Sends bulk/control commands directly.
	/// \param command The command as string (Has to be parsed).
	/// \return 0 on success, -1 on unknown command, -2 on syntax error.
	int Control::stringCommand(QString command) {
		if(!this->device->isConnected())
			return -3;

		return -1;
	}
#endif

}
