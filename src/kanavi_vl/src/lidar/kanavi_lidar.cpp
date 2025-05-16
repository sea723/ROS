#include "kanavi_lidar.h"

/**
 * @brief Construct a new kanavi lidar::kanavi lidar object
 *
 * @param model_ LiDAR Model ref include/common.h
 */
kanavi_lidar::kanavi_lidar(int model_)
{
<<<<<<< HEAD
	datagram_ = nullptr;
	try {
		datagram_ = new kanaviDatagram(model_);
		if (!datagram_) {
			throw std::runtime_error("Failed to allocate datagram");
		}

		checked_onGoing = false;
		checked_pares_end = false;
		memset(checked_ch_r4, false, 4);

		checked_ch0_inputed = false;
		checked_model = -1;
		total_size = 0;

		// Initialize vectors based on model
		switch (model_)
		{
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
			datagram_->raw_buf.resize(KANAVI::R2::VERTICAL_CHANNEL);
			datagram_->len_buf.resize(KANAVI::R2::VERTICAL_CHANNEL);
			for (auto& buf : datagram_->len_buf) {
				buf.reserve(KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_DATA_CNT);
			}
			break;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
			datagram_->raw_buf.resize(KANAVI::R4::VERTICAL_CHANNEL);
			datagram_->len_buf.resize(KANAVI::R4::VERTICAL_CHANNEL);
			for (auto& buf : datagram_->len_buf) {
				buf.reserve(KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_DATA_CNT);
			}
			break;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
			datagram_->raw_buf.resize(KANAVI::R270::VERTICAL_CHANNEL);
			datagram_->len_buf.resize(KANAVI::R270::VERTICAL_CHANNEL);
			for (auto& buf : datagram_->len_buf) {
				buf.reserve(KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_DATA_CNT);
			}
			break;
		default:
			throw std::runtime_error("Invalid model type");
		}
	} catch (const std::exception& e) {
		printf("[LiDAR] Error in constructor: %s\n", e.what());
		if (datagram_) {
			delete datagram_;
			datagram_ = nullptr;
		}
		throw;
	}
=======
	datagram_ = new kanaviDatagram(model_);
	checked_onGoing = false;
	checked_pares_end = false;
	memset(checked_ch_r4, false, 4);

	checked_ch0_inputed = false;

	checked_model = -1;

	total_size = 0;
>>>>>>> a5b66cff43279b37025a24310b4ee71a65cbfcf2
}

/**
 * @brief Destroy the kanavi lidar::kanavi lidar object
 *
 */
kanavi_lidar::~kanavi_lidar()
{
<<<<<<< HEAD
	if (datagram_ != nullptr)
	{
		delete datagram_;
		datagram_ = nullptr;
	}
=======
>>>>>>> a5b66cff43279b37025a24310b4ee71a65cbfcf2
}

/**
 * @brief check LiDAR Sensor
 *
 * @param data
 * @return int
 */
int kanavi_lidar::classification(const std::vector<u_char> &data)
{
	if ((data[KANAVI::COMMON::PROTOCOL_POS::HEADER] & 0xFF) == KANAVI::COMMON::PROTOCOL_VALUE::HEADER) // industrial header detected
	{
		u_char indus_M = data[KANAVI::COMMON::PROTOCOL_POS::PRODUCT_LINE];
		switch (indus_M)
		{
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
			printf("********R2**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
			printf("********R4**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
			printf("********R270**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270;
		default:
			return -1;
		}
	}

	return -1;
}

int kanavi_lidar::process(const std::vector<u_char> &data)
{
<<<<<<< HEAD
	if (!datagram_) {
		printf("[LiDAR] Datagram is not initialized\n");
		return -1;
	}

	if (data.empty()) {
		printf("[LiDAR] Empty data received\n");
		return -1;
	}

	// header value Check in data
	if ((data[KANAVI::COMMON::PROTOCOL_POS::HEADER] & 0xFF) != KANAVI::COMMON::PROTOCOL_VALUE::HEADER) {
		printf("[LiDAR] Invalid header\n");
		return -1;
	}

	// u_char ch_ = data[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
	u_char mode = data[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];

	// check Model
	if (!checked_ch0_inputed) {
		checked_model = classification(data);
		if (checked_model == -1) {
			printf("[LiDAR] Classification Error!\n");
			return -1;
		}

		if (checked_model != datagram_->model) {
			printf("[LiDAR] Model not Matched\n");
			return -1;
		}
		checked_ch0_inputed = true;
	}

	// Process data based on mode
	if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA) {
		printf("---------KANAVI PROCESS------------\n");
		
		try {
			// Process the data directly
			switch (checked_model) {
				case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
					r2(data, datagram_);
					break;
				case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
					r4(data, datagram_);
					break;
				case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
					r270(data, datagram_);
					break;
				default:
					printf("[LiDAR] Unknown model\n");
					return -1;
			}
			
			checked_pares_end = true;
			return 1;
		} catch (const std::exception& e) {
			printf("[LiDAR] Error in process: %s\n", e.what());
			return -1;
		}
	}

	return 0;
=======
	// r270데이터가 끊어져서 들어오므로 합칠 필요가 있음.
	// hedaer checker
	bool checked_h = false;

	// header value Check in data
	if ((data[KANAVI::COMMON::PROTOCOL_POS::HEADER] & 0xFF) == KANAVI::COMMON::PROTOCOL_VALUE::HEADER)
	{
		checked_h = true;

		u_char ch_ = data[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

		// check Model
		if (!checked_ch0_inputed)
		{
			checked_model = classification(data);
			if (checked_model == -1)
			{
				perror("LiDAR Classification Error!");
				return -1;
			}

			// check Model, one more time
			if (checked_model != datagram_->model)
			{
				perror("LiDAR Model not Matched");
				return -1;
			}
		}

		if (ch_ == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			printf("---------KANAVI PROCESS------------\n");

			checked_ch0_inputed = true;
			checked_pares_end = false;
			temp_buf_.clear();
			switch (checked_model)
			{
			case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
				/* code */
				total_size = KANAVI::R2::DATASIZE * 2;
				break;
			case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
				total_size = KANAVI::R270::DATASIZE;
				/* code */
				break;
			case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
				total_size = KANAVI::R4::DATASIZE * 4;
				/* code */
				break;
			default:
				break;
			}
		}
	}

	if (checked_ch0_inputed)
	{
		// 0번 채널이 들어온 이후부터 데이터 축적
		temp_buf_.insert(temp_buf_.end(), data.begin(), data.end());
		printf("[LiDAR] Processing...\n");
	}

	// 채널이 많으면 채널을 확인해서 전부 취합되었는지 확인
	if (temp_buf_.size() == total_size)
	{
		parse(temp_buf_);
		checked_ch0_inputed = false;
	}
>>>>>>> a5b66cff43279b37025a24310b4ee71a65cbfcf2
}

std::string kanavi_lidar::getLiDARModel()
{
	return std::string();
}

void kanavi_lidar::parse(const std::vector<u_char> &data)
{
<<<<<<< HEAD
	if (data.size() < KANAVI::COMMON::PROTOCOL_POS::RAWDATA_START)
	{
		printf("[LiDAR] Invalid data size\n");
		return;
	}

	switch (datagram_->model)
	{
	case KANAVI::COMMON::PROTOCOL_VALUE::R2:
		r2(data, datagram_);
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::R4:
		r4(data, datagram_);
=======
	switch (datagram_->model)
	{
	case KANAVI::COMMON::PROTOCOL_VALUE::R2:
		for (int i = 0; i < 2; i++)
		{
			auto start = data.begin() + i * KANAVI::R2::DATASIZE;
			auto end = start + KANAVI::R2::DATASIZE;

			if (end <= data.end())
			{ // 벡터 범위 체크
				r2(std::vector<u_char>(start, end), datagram_);
			}
			else
			{
				// 데이터 부족 시 처리 로직
				std::cerr << "Not enough data for chunk " << i << std::endl;
			}
		}
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::R4:
		for (int i = 0; i < 4; i++)
		{
			auto start = data.begin() + i * KANAVI::R4::DATASIZE;
			auto end = start + KANAVI::R4::DATASIZE;

			if (end <= data.end())
			{ // 벡터 범위 체크
				r4(std::vector<u_char>(start, end), datagram_);
			}
			else
			{
				// 데이터 부족 시 처리 로직
				std::cerr << "Not enough data for chunk " << i << std::endl;
			}
		}
>>>>>>> a5b66cff43279b37025a24310b4ee71a65cbfcf2
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::R270:
		r270(data, datagram_);
		break;
	}
}

void kanavi_lidar::r2(const std::vector<u_char> &input, kanaviDatagram *output)
{
	// u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	// u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	// if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	// {
	// 	// check ch 0 data input
	// 	if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
	// 	{
	// 		// set specification
	// 		{
	// 			output->v_resolution = KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_RESOLUTION;
	// 			output->h_resolution = KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_RESOLUTION;
	// 		}
	// 	}
	// 	parseLength(input, output, static_cast<int>(ch & 0x0F)); // convert byte to length
	// }
}

void kanavi_lidar::r4(const std::vector<u_char> &input, kanaviDatagram *output)
{
<<<<<<< HEAD
	if (!output) {
		printf("[LiDAR] Invalid output pointer in r4\n");
		return;
	}

	if (input.size() < KANAVI::COMMON::PROTOCOL_POS::RAWDATA_START) {
		printf("[LiDAR] Invalid input size in r4\n");
		return;
	}

	try {
		u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
		int channel = static_cast<int>(ch & 0x0F);

		// Validate channel number
		if (channel >= KANAVI::R4::VERTICAL_CHANNEL) {
			printf("[LiDAR] Invalid channel number: %d\n", channel);
			return;
		}

		// Ensure len_buf is properly initialized
		if (output->len_buf.empty()) {
			printf("[LiDAR] len_buf not properly initialized for channel %d\n", channel);
			return;
		}

		// set specification for any channel
		output->v_resolution = KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION;
		output->h_resolution = KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;

		// process the current channel data
		parseLength(input, output, channel);
	} catch (const std::exception& e) {
		printf("[LiDAR] Error in r4: %s\n", e.what());
=======
	u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		// check ch 0 data input
		if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			// set specification
			{
				output->v_resolution = KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION;
				output->h_resolution = KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
			}
		}
		// printf("[LiDAR] Process CH : %d\n", ch & 0x0F);
		checked_ch_r4[ch & 0x0F] = true;
		parseLength(input, output, static_cast<int>(ch & 0x0F)); // convert byte to length

		if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_3)
		{
			// checked_onGoing = true;
			if (checked_ch_r4[0] && checked_ch_r4[1] && checked_ch_r4[2] && checked_ch_r4[3])
			{
				memset(checked_ch_r4, false, 4);
				checked_pares_end = true;
			}
			else
			{
				memset(checked_ch_r4, false, 4);
				printf("[LiDAR] Align ERROR#####\n");
			}
		}
>>>>>>> a5b66cff43279b37025a24310b4ee71a65cbfcf2
	}
}

void kanavi_lidar::r270(const std::vector<u_char> &input, kanaviDatagram *output)
{
	u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		// check ch 0 data input
		if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			// set specification
			{
				output->v_resolution = KANAVI::COMMON::SPECIFICATION::R270::VERTICAL_RESOLUTION;
				output->h_resolution = KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_RESOLUTION;
			}

			parseLength(input, output, static_cast<int>(ch & 0x0F)); // convert byte to length
			checked_pares_end = true;
		}
	}
}

void kanavi_lidar::parseLength(const std::vector<u_char> &input, kanaviDatagram *output, int ch)
{
<<<<<<< HEAD
	if (!output) {
		printf("[LiDAR] Invalid output pointer in parseLength\n");
		return;
	}

	if (ch >= KANAVI::R4::VERTICAL_CHANNEL) {
		printf("[LiDAR] Invalid channel number: %d\n", ch);
		return;
	}

	if (static_cast<size_t>(ch) >= output->len_buf.size()) {
		printf("[LiDAR] Channel index out of range: %d\n", ch);
		return;
	}

	try {
		// Calculate expected size
		int start = KANAVI::COMMON::PROTOCOL_POS::RAWDATA_START;
		int end = input.size() - KANAVI::COMMON::PROTOCOL_SIZE::CHECKSUM;
		int expected_size = (end - start) / 2;

		if (expected_size <= 0) {
			printf("[LiDAR] Invalid data size: start=%d, end=%d\n", start, end);
			return;
		}

		// Create new vector with proper size
		std::vector<float> len_;
		len_.reserve(expected_size);

		// Process data
		for (int i = start; i < end; i += 2) {
			if (i + 1 >= static_cast<int>(input.size())) {
				printf("[LiDAR] Data size error at index %d\n", i);
				break;
			}
			float up = input[i];
			float low = input[i + 1];
			float len = up + low / 100; // convert 2 byte to length[m]
			len_.push_back(len);
		}

		// Verify the processed data
		if (len_.empty()) {
			printf("[LiDAR] No valid data processed for channel %d\n", ch);
			return;
		}

		// Update the output buffer
		output->len_buf[ch] = std::move(len_);
	} catch (const std::exception& e) {
		printf("[LiDAR] Error in parseLength: %s\n", e.what());
	}
=======
	std::vector<float> len_;

	int start = KANAVI::COMMON::PROTOCOL_POS::RAWDATA_START;
	int end = input.size() - KANAVI::COMMON::PROTOCOL_SIZE::CHECKSUM;

	float up = 0;
	float low = 0;
	float len = 0;

	for (int i = start; i < end; i += 2)
	{
		up = input[i];
		low = input[i + 1];
		len = up + low / 100; // convert 2 byte to length[m]
		len_.push_back(len);
	}
	output->len_buf[ch] = len_;
	// checked_pares_end = true;
>>>>>>> a5b66cff43279b37025a24310b4ee71a65cbfcf2
}

bool kanavi_lidar::checkedProcessEnd()
{
	return checked_pares_end;
}

kanaviDatagram kanavi_lidar::getDatagram()
{
	return *datagram_;
}