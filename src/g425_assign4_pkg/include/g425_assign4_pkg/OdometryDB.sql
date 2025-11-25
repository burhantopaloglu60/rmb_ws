-- phpMyAdmin SQL Dump
-- version 5.2.1deb3
-- https://www.phpmyadmin.net/
--
-- Host: localhost:3306
-- Generation Time: Nov 25, 2025 at 02:32 PM
-- Server version: 10.11.13-MariaDB-0ubuntu0.24.04.1
-- PHP Version: 8.3.6

SET SQL_MODE = "NO_AUTO_VALUE_ON_ZERO";
START TRANSACTION;
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;

--
-- Database: `OdometryDB`
--

-- --------------------------------------------------------

--
-- Table structure for table `IMU_sim_acceleration`
--

CREATE TABLE `IMU_sim_acceleration` (
  `x` float NOT NULL,
  `y` float NOT NULL,
  `z` float NOT NULL,
  `yaw_z` float NOT NULL,
  `Timestamp` timestamp NOT NULL DEFAULT current_timestamp()
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `IMU_sim_pos`
--

CREATE TABLE `IMU_sim_pos` (
  `x` float NOT NULL,
  `y` float NOT NULL,
  `z` float NOT NULL,
  `yaw_z` float NOT NULL,
  `Timestamp` timestamp NOT NULL DEFAULT current_timestamp()
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `IMU_sim_velocity`
--

CREATE TABLE `IMU_sim_velocity` (
  `x` float NOT NULL,
  `y` float NOT NULL,
  `z` float NOT NULL,
  `yaw_z` float NOT NULL,
  `Timestamp` timestamp NOT NULL DEFAULT current_timestamp()
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `Mecanum_pos`
--

CREATE TABLE `Mecanum_pos` (
  `x` float NOT NULL,
  `y` float NOT NULL,
  `z` float NOT NULL,
  `yaw_z` float NOT NULL,
  `Timestamp` timestamp NOT NULL DEFAULT current_timestamp()
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `Mecanum_velocity`
--

CREATE TABLE `Mecanum_velocity` (
  `x` float NOT NULL,
  `y` float NOT NULL,
  `z` float NOT NULL,
  `yaw_z` float NOT NULL,
  `Timestamp` timestamp NOT NULL DEFAULT current_timestamp()
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Indexes for dumped tables
--

--
-- Indexes for table `IMU_sim_acceleration`
--
ALTER TABLE `IMU_sim_acceleration`
  ADD UNIQUE KEY `Timestamp` (`Timestamp`);

--
-- Indexes for table `IMU_sim_pos`
--
ALTER TABLE `IMU_sim_pos`
  ADD UNIQUE KEY `Timestamp` (`Timestamp`);

--
-- Indexes for table `IMU_sim_velocity`
--
ALTER TABLE `IMU_sim_velocity`
  ADD UNIQUE KEY `Timestamp` (`Timestamp`);

--
-- Indexes for table `Mecanum_pos`
--
ALTER TABLE `Mecanum_pos`
  ADD UNIQUE KEY `Timestamp` (`Timestamp`);

--
-- Indexes for table `Mecanum_velocity`
--
ALTER TABLE `Mecanum_velocity`
  ADD UNIQUE KEY `Timestamp` (`Timestamp`);
COMMIT;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
