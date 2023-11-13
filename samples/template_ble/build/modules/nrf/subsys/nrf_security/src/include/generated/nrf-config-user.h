/*
 * Copyright (c) 2021 Nordic Semiconductor
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 */

#ifndef PSA_CRYPTO_CONFIG_H
#define PSA_CRYPTO_CONFIG_H


#define PSA_CORE_OBERON                                    1

#define PSA_WANT_GENERATE_RANDOM                           1
#define PSA_WANT_ALG_CTR_DRBG                              1
/* #undef PSA_WANT_ALG_HMAC_DRBG */
/*
 * CBC-MAC is not yet supported via the PSA API in Mbed TLS.
 */
//#define PSA_WANT_ALG_CBC_MAC
/* #undef PSA_WANT_ALG_CBC_NO_PADDING */
/* #undef PSA_WANT_ALG_CBC_PKCS7 */
#define PSA_WANT_ALG_CCM                                   1
#define PSA_WANT_ALG_CMAC                                  1
/* #undef PSA_WANT_ALG_CFB */
/* #undef PSA_WANT_ALG_CHACHA20_POLY1305 */
#define PSA_WANT_ALG_CMAC                                  1
#define PSA_WANT_ALG_CTR                                   1
#define PSA_WANT_ALG_DETERMINISTIC_ECDSA                   1
#define PSA_WANT_ALG_ECB_NO_PADDING                        1
#define PSA_WANT_ALG_ECDH                                  1
#define PSA_WANT_ALG_ECDSA                                 1
#define PSA_WANT_ALG_GCM                                   1
/* #undef PSA_WANT_ALG_HKDF */
#define PSA_WANT_ALG_HMAC                                  1
/* #undef PSA_WANT_ALG_MD5 */
/* #undef PSA_WANT_ALG_OFB */
/* #undef PSA_WANT_ALG_PBKDF2_HMAC */
/* #undef PSA_WANT_ALG_PBKDF2_AES_CMAC_PRF_128 */
/* #undef PSA_WANT_ALG_RIPEMD160 */
/* #undef PSA_WANT_ALG_RSA_OAEP */
/* #undef PSA_WANT_ALG_RSA_PKCS1V15_CRYPT */
/* #undef PSA_WANT_ALG_RSA_PKCS1V15_SIGN */
/* #undef PSA_WANT_ALG_RSA_PSS */
#define PSA_WANT_ALG_SHA_1                                 1
#define PSA_WANT_ALG_SHA_224                               1
#define PSA_WANT_ALG_SHA_256                               1
/* #undef PSA_WANT_ALG_SHA_384 */
#define PSA_WANT_ALG_SHA_512                               1
#define PSA_WANT_ALG_STREAM_CIPHER                         1
/* #undef PSA_WANT_ALG_JPAKE */
/* #undef PSA_WANT_ALG_SPAKE2P */
/* #undef PSA_WANT_ALG_SRP_6 */
/* #undef PSA_WANT_ALG_TLS12_PRF */
/* #undef PSA_WANT_ALG_TLS12_PSK_TO_MS */
/* #undef PSA_WANT_ALG_TLS12_ECJPAKE_TO_PMS */
/* #undef PSA_WANT_ALG_XTS */

/* #undef PSA_WANT_ECC_BRAINPOOL_P_R1_256 */
/* #undef PSA_WANT_ECC_BRAINPOOL_P_R1_384 */
/* #undef PSA_WANT_ECC_BRAINPOOL_P_R1_512 */
#define PSA_WANT_ECC_MONTGOMERY_255                        1
/* #undef PSA_WANT_ECC_MONTGOMERY_448 */
/* #undef PSA_WANT_ECC_SECP_K1_192 */
/*
 * SECP224K1 is buggy via the PSA API in Mbed TLS
 * (https://github.com/ARMmbed/mbedtls/issues/3541). Thus, do not enable it by
 * default.
 */
/* #undef PSA_WANT_ECC_SECP_K1_224 */
/* #undef PSA_WANT_ECC_SECP_K1_256 */
/* #undef PSA_WANT_ECC_SECP_R1_192 */
/* #undef PSA_WANT_ECC_SECP_R1_224 */
#define PSA_WANT_ECC_SECP_R1_256                           1
/* #undef PSA_WANT_ECC_SECP_R1_384 */
/* #undef PSA_WANT_ECC_SECP_R1_521 */

/* #undef PSA_WANT_KEY_TYPE_DERIVE */
#define PSA_WANT_KEY_TYPE_HMAC                             1
/* #undef PSA_WANT_KEY_TYPE_PASSWORD */
/* #undef PSA_WANT_KEY_TYPE_PASSWORD_HASH */
/* #undef PSA_WANT_KEY_TYPE_PEPPER */
/* #undef PSA_WANT_KEY_TYPE_RAW_DATA */

#define PSA_WANT_KEY_TYPE_AES                              1
/* #undef PSA_WANT_KEY_TYPE_ARIA */
/* #undef PSA_WANT_KEY_TYPE_CAMELLIA */
#define PSA_WANT_KEY_TYPE_CHACHA20                         1
/* #undef PSA_WANT_KEY_TYPE_DES */
#define PSA_WANT_KEY_TYPE_ECC_KEY_PAIR                     1
#define PSA_WANT_KEY_TYPE_ECC_PUBLIC_KEY                   1
/* #undef PSA_WANT_KEY_TYPE_RSA_KEY_PAIR */
/* #undef PSA_WANT_KEY_TYPE_RSA_PUBLIC_KEY */

/*
 * Accelerated key types
 */
/* #undef MBEDTLS_PSA_ACCEL_KEY_TYPE_SUPPORT */
/* #undef MBEDTLS_PSA_ACCEL_KEY_TYPE_ECC_KEY_PAIR */
/* #undef MBEDTLS_PSA_ACCEL_KEY_TYPE_ECC_PUBLIC_KEY */
/* #undef MBEDTLS_PSA_ACCEL_KEY_TYPE_RSA_KEY_PAIR */
/* #undef MBEDTLS_PSA_ACCEL_KEY_TYPE_RSA_PUBLIC_KEY */
/* #undef MBEDTLS_PSA_ACCEL_KEY_TYPE_AES */
/* #undef MBEDTLS_PSA_ACCEL_KEY_TYPE_CHACHA20 */

/*
 * nrf_cc3xx_platform driver configurations
 */
#define PSA_NEED_CC3XX_CTR_DRBG_DRIVER                     1
/* #undef PSA_NEED_CC3XX_HMAC_DRBG_DRIVER */

/*
 * PSA driver configurations
 */
#define PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT 1

#define PSA_MAX_RSA_KEY_BITS 3072

/*
 * nrf_cc3xx driver configurations
 */
/* #undef PSA_NEED_CC3XX_AEAD_DRIVER */
/* #undef PSA_NEED_CC3XX_ASYMMETRIC_DRIVER */
/* #undef PSA_NEED_CC3XX_CIPHER_DRIVER */
/* #undef PSA_NEED_CC3XX_ECDH_DRIVER */
/* #undef PSA_NEED_CC3XX_ENTROPY_DRIVER */
/* #undef PSA_NEED_CC3XX_HASH_DRIVER */
/* #undef PSA_NEED_CC3XX_KEY_PAIR_DRIVER */
/* #undef PSA_NEED_CC3XX_MAC_DRIVER */
/* #undef PSA_NEED_CC3XX_SIGNATURE_DRIVER */

/*
 * nrf_oberon driver configurations
 */
#define PSA_NEED_OBERON_AEAD_DRIVER                        1
#define PSA_NEED_OBERON_CIPHER_DRIVER                      1
/* #undef PSA_NEED_OBERON_CTR_DRBG_DRIVER */
#define PSA_NEED_OBERON_ECDH_DRIVER                        1
#define PSA_NEED_OBERON_ECDSA_DRIVER                       1
#define PSA_NEED_OBERON_HASH_DRIVER                        1
/* #undef PSA_NEED_OBERON_HMAC_DRBG_DRIVER */
/* #undef PSA_NEED_OBERON_JPAKE_DRIVER */
#define PSA_NEED_OBERON_KDF_DRIVER                         1
#define PSA_NEED_OBERON_KEY_PAIR_DRIVER                    1
#define PSA_NEED_OBERON_MAC_DRIVER                         1
/* #undef PSA_NEED_OBERON_RSA_CRYPT */
/* #undef PSA_NEED_OBERON_RSA_DRIVER */
/* #undef PSA_NEED_OBERON_RSA_SIGN */
/* #undef PSA_NEED_OBERON_SPAKE2P_DRIVER */
/* #undef PSA_NEED_OBERON_SRP_DRIVER */

/* #undef PSA_NEED_OBERON_AES_CBC_NO_PADDING */
/* #undef PSA_NEED_OBERON_AES_CBC_PKCS7 */
#define PSA_NEED_OBERON_AES_CCM                            1
/* #undef PSA_NEED_OBERON_AES_CCM_STAR_NO_TAG */
#define PSA_NEED_OBERON_AES_CTR                            1
#define PSA_NEED_OBERON_AES_ECB_NO_PADDING                 1
#define PSA_NEED_OBERON_AES_GCM                            1
#define PSA_NEED_OBERON_CHACHA20                           1
/* #undef PSA_NEED_OBERON_CHACHA20_POLY1305 */
#define PSA_NEED_OBERON_CMAC                               1
#define PSA_NEED_OBERON_DETERMINISTIC_ECDSA                1
/* #undef PSA_NEED_OBERON_ECDH_P224 */
#define PSA_NEED_OBERON_ECDH_P256                          1
/* #undef PSA_NEED_OBERON_ECDH_P384 */
#define PSA_NEED_OBERON_ECDH_X25519                        1
#define PSA_NEED_OBERON_ECDSA_ED25519                      1
/* #undef PSA_NEED_OBERON_ECDSA_P224 */
#define PSA_NEED_OBERON_ECDSA_P256                         1
/* #undef PSA_NEED_OBERON_ECDSA_P384 */
/* #undef PSA_NEED_OBERON_ECJPAKE_TO_PMS */
/* #undef PSA_NEED_OBERON_HKDF */
/* #undef PSA_NEED_OBERON_HKDF_EXPAND */
/* #undef PSA_NEED_OBERON_HKDF_EXTRACT */
#define PSA_NEED_OBERON_HMAC                               1
#define PSA_NEED_OBERON_KEY_PAIR_25519                     1
#define PSA_NEED_OBERON_KEY_PAIR_ED25519                   1
/* #undef PSA_NEED_OBERON_KEY_PAIR_P224 */
#define PSA_NEED_OBERON_KEY_PAIR_P256                      1
/* #undef PSA_NEED_OBERON_KEY_PAIR_P384 */
/* #undef PSA_NEED_OBERON_KEY_PAIR_P521 */
#define PSA_NEED_OBERON_KEY_PAIR_SECP                      1
#define PSA_NEED_OBERON_KEY_PAIR_X25519                    1
#define PSA_NEED_OBERON_PBKDF2_AES_CMAC_PRF_128            1
/* #undef PSA_NEED_OBERON_PBKDF2_HMAC */
#define PSA_NEED_OBERON_RANDOMIZED_ECDSA                   1
/* #undef PSA_NEED_OBERON_RSA_KEY_SIZE_1024 */
/* #undef PSA_NEED_OBERON_RSA_KEY_SIZE_1536 */
/* #undef PSA_NEED_OBERON_RSA_KEY_SIZE_2048 */
/* #undef PSA_NEED_OBERON_RSA_KEY_SIZE_3072 */
/* #undef PSA_NEED_OBERON_RSA_KEY_SIZE_4096 */
/* #undef PSA_NEED_OBERON_RSA_KEY_SIZE_6144 */
/* #undef PSA_NEED_OBERON_RSA_KEY_SIZE_8192 */
/* #undef PSA_NEED_OBERON_RSA_OAEP */
/* #undef PSA_NEED_OBERON_RSA_PKCS1V15_CRYPT */
/* #undef PSA_NEED_OBERON_RSA_PKCS1V15_SIGN */
/* #undef PSA_NEED_OBERON_RSA_PSS */
#define PSA_NEED_OBERON_SHA_1                              1
#define PSA_NEED_OBERON_SHA_224                            1
#define PSA_NEED_OBERON_SHA_256                            1
/* #undef PSA_NEED_OBERON_SHA_384 */
#define PSA_NEED_OBERON_SHA_512                            1
/* #undef PSA_NEED_OBERON_TLS12_PRF */
/* #undef PSA_NEED_OBERON_TLS12_PSK_TO_MS */
/* #undef PSA_NEED_OBERON_JPAKE_DRIVER */
/* #undef PSA_NEED_OBERON_SPAKE2P_DRIVER */
/* #undef PSA_NEED_OBERON_SRP_DRIVER */

/* #undef PSA_NEED_ZEPHYR_ENTROPY_DRIVER */

/* Nordic specific */
/* #undef PSA_NATIVE_ITS */
/* #undef PSA_NATIVE_ITS_BACKEND_ZEPHYR */
/* #undef PSA_CRYPTO_SECURE */
/* #undef PSA_CRYPTO_DRIVER_ALG_PRNG_TEST */

/* PSA and drivers */
#define MBEDTLS_PSA_CRYPTO_C
#define MBEDTLS_USE_PSA_CRYPTO
/* #undef MBEDTLS_PSA_CRYPTO_STORAGE_C */
/* MBEDTLS_PSA_CRYPTO_DRIVERS is defined to 1 by TF-M's build system. */
#define MBEDTLS_PSA_CRYPTO_DRIVERS                         1
#define MBEDTLS_PSA_CRYPTO_CLIENT
#define MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG

/* TF-M */
/* #undef MBEDTLS_PSA_CRYPTO_SPM */
/* Avoid redefinition as TF-M defines this on the command line */
#ifndef MBEDTLS_PSA_CRYPTO_KEY_ID_ENCODES_OWNER
/* #undef MBEDTLS_PSA_CRYPTO_KEY_ID_ENCODES_OWNER */
#endif

/* Platform */
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_MEMORY_BUFFER_ALLOC_C

/* Platform configurations for _ALT defines */
/* #undef MBEDTLS_PLATFORM_EXIT_ALT */
/* #undef MBEDTLS_PLATFORM_FPRINTF_ALT */
/* #undef MBEDTLS_PLATFORM_PRINTF_ALT */
/* #undef MBEDTLS_PLATFORM_SNPRINTF_ALT */
/* #undef MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT */
#define MBEDTLS_ENTROPY_HARDWARE_ALT
/* #undef MBEDTLS_THREADING_C */
/* #undef MBEDTLS_THREADING_ALT */
#define MBEDTLS_PLATFORM_ZEROIZE_ALT

/* Legacy configurations for _ALT defines */
#define MBEDTLS_AES_SETKEY_ENC_ALT
#define MBEDTLS_AES_SETKEY_DEC_ALT
#define MBEDTLS_AES_ENCRYPT_ALT
#define MBEDTLS_AES_DECRYPT_ALT
/* #undef MBEDTLS_AES_ALT */
/* #undef MBEDTLS_CMAC_ALT */
/* #undef MBEDTLS_CCM_ALT */
/* #undef MBEDTLS_GCM_ALT */
#define MBEDTLS_CHACHA20_ALT
#define MBEDTLS_POLY1305_ALT
/* #undef MBEDTLS_CHACHAPOLY_ALT */
/* #undef MBEDTLS_DHM_ALT */
/* #undef MBEDTLS_ECP_ALT */
#define MBEDTLS_ECDH_GEN_PUBLIC_ALT
#define MBEDTLS_ECDH_COMPUTE_SHARED_ALT
#define MBEDTLS_ECDSA_GENKEY_ALT
#define MBEDTLS_ECDSA_SIGN_ALT
#define MBEDTLS_ECDSA_VERIFY_ALT
#define MBEDTLS_ECJPAKE_ALT
/* #undef MBEDTLS_RSA_ALT */
#define MBEDTLS_SHA1_ALT
#define MBEDTLS_SHA224_ALT
#define MBEDTLS_SHA256_ALT
/* #undef MBEDTLS_SHA384_ALT */
/* #undef MBEDTLS_SHA512_ALT */

/* Legacy configuration for RNG */
#define MBEDTLS_ENTROPY_FORCE_SHA256
#define MBEDTLS_ENTROPY_MAX_SOURCES                        1
#define MBEDTLS_NO_PLATFORM_ENTROPY

/* Legacy configurations for mbed TLS APIs */
#define MBEDTLS_CIPHER_C
//#define MBEDTLS_PK_C                                       
//#define MBEDTLS_PK_WRITE_C                                 
#define MBEDTLS_MD_C

/* Max curve bits for PSA APIs */
/* #undef PSA_VENDOR_ECC_MAX_CURVE_BITS */


/* TLS/DTLS configurations */
/* #undef MBEDTLS_SSL_ALL_ALERT_MESSAGES */
/* #undef MBEDTLS_SSL_DTLS_CONNECTION_ID */
/* #undef MBEDTLS_SSL_CONTEXT_SERIALIZATION */
/* #undef MBEDTLS_SSL_DEBUG_ALL */
/* #undef MBEDTLS_SSL_ENCRYPT_THEN_MAC */
/* #undef MBEDTLS_SSL_EXTENDED_MASTER_SECRET */
/* #undef MBEDTLS_SSL_KEEP_PEER_CERTIFICATE */
/* #undef MBEDTLS_SSL_RENEGOTIATION */
/* #undef MBEDTLS_SSL_MAX_FRAGMENT_LENGTH */
/* #undef MBEDTLS_SSL_PROTO_TLS1_2 */
/* #undef MBEDTLS_SSL_PROTO_DTLS */
/* #undef MBEDTLS_SSL_ALPN */
/* #undef MBEDTLS_SSL_DTLS_ANTI_REPLAY */
/* #undef MBEDTLS_SSL_DTLS_HELLO_VERIFY */
/* #undef MBEDTLS_SSL_DTLS_SRTP */
/* #undef MBEDTLS_SSL_DTLS_CLIENT_PORT_REUSE */
/* #undef MBEDTLS_SSL_SESSION_TICKETS */
/* #undef MBEDTLS_SSL_EXPORT_KEYS */
/* #undef MBEDTLS_SSL_SERVER_NAME_INDICATION */
/* #undef MBEDTLS_SSL_VARIABLE_BUFFER_LENGTH */
/* #undef MBEDTLS_SSL_CACHE_C */
/* #undef MBEDTLS_SSL_TICKET_C */
/* #undef MBEDTLS_SSL_CLI_C */
/* #undef MBEDTLS_SSL_COOKIE_C */
/* #undef MBEDTLS_SSL_SRV_C */
/* #undef MBEDTLS_SSL_TLS_C */
/* #undef MBEDTLS_SSL_IN_CONTENT_LEN */
/* #undef MBEDTLS_SSL_OUT_CONTENT_LEN */
/* #undef MBEDTLS_SSL_CIPHERSUITES */

/* #undef MBEDTLS_X509_RSASSA_PSS_SUPPORT */
/* #undef MBEDTLS_X509_USE_C */
/* #undef MBEDTLS_X509_CRT_PARSE_C */
/* #undef MBEDTLS_X509_CRL_PARSE_C */
/* #undef MBEDTLS_X509_CSR_PARSE_C */
/* #undef MBEDTLS_X509_CREATE_C */
/* #undef MBEDTLS_X509_CRT_WRITE_C */
/* #undef MBEDTLS_X509_CSR_WRITE_C */
/* #undef MBEDTLS_X509_REMOVE_INFO */

/* #undef MBEDTLS_KEY_EXCHANGE_PSK_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_RSA_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED */
/* #undef MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED */

/* Controlling some MPI sizes */
#define MBEDTLS_MPI_WINDOW_SIZE       6 /**< Maximum window size used. */
#define MBEDTLS_MPI_MAX_SIZE          256 /**< Maximum number of bytes for usable MPIs. */

#include <psa/core_unsupported_ciphers_check.h>

#endif /* PSA_CRYPTO_CONFIG_H */
