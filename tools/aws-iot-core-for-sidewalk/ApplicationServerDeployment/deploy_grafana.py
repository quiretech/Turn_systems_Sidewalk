# Copyright 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

"""
Scripts deploys Grafana stack.
It also creates Grafana workspace and configures dashboard.
"""

import boto3

from constants.GrafanaConstants import *
from libs.cloud_formation_client import CloudFormationClient
from libs.config import Config
from libs.grafana_client import GrafanaClient
from libs.identity_store_client import IdentityStoreClient
from libs.utils import *
from libs.iot_wireless_client import IoTWirelessClient


# -----------------
# Read config file
# -----------------
config = Config()


# --------------------
# Ask user to proceed
# --------------------
log_info('Arguments to be used during the SidewalkSampleApplication deployment:')
log_info(f'\tCONFIG_PROFILE: {config.aws_profile}')
log_info(f'\tREGION: {config.region_name}')
msg = f"{config.identity_store_id}" if config.identity_store_id else "<empty> - no users will be created"
log_info(f'\tIDENTITY_STORE_ID: {msg}')
if config.identity_store_id and config.identity_center_users:
    log_info(f'\tIDENTITY_CENTER_USERS: ')
    for idx, user in enumerate(config.identity_center_users):
        log_info(
            f'\t\t{idx + 1}.'
            f'{user.username}'
            f', {user.firstname}'
            f', {user.lastname}')
elif config.identity_store_id:
    log_info(f'\tIDENTITY_CENTER_USERS: no valid entries found; no users will be created')
else:
    pass
if config.interactive_mode:
    log_info(f'Proceed with stack creation?')
    confirm()


# -------------------------------------------------------------
# Create boto3 session using given profile and service clients
# -------------------------------------------------------------
session = boto3.Session(profile_name=config.aws_profile, region_name=config.region_name)
cf_client = CloudFormationClient(session)
grafana_client = GrafanaClient(session)
idstore_client = IdentityStoreClient(session, config.identity_store_id)
wireless_client = IoTWirelessClient(session)

# ------------------------------------
# Enable Sidewalk event notifications
# ------------------------------------
wireless_client.enable_notifications()


# ---------------------------------------------------
# Check if given Sidewalk destination already exists
# ---------------------------------------------------
sid_dest_already_exists = wireless_client.check_if_destination_exists(name=config.sid_dest_name)


# -----------------------------
# Read CloudFormation template
# -----------------------------
stack_path = Path(__file__).parent.joinpath('template', 'SidewalkGrafanaStack.yaml')
stack = read_file(stack_path)


# --------------------------------------
# Trigger CloudFormation stack creation
# --------------------------------------
cf_client.create_stack(
    template=stack,
    stack_name=STACK_NAME,
    sid_dest=config.sid_dest_name,
    dest_exists=sid_dest_already_exists,
    tag=TAG
)


# ------------------------------------------------------------------------
# Update given Sidewalk destination (only if destination already existed)
# ------------------------------------------------------------------------
if sid_dest_already_exists:
    wireless_client.update_existing_destination(
        dest_name=config.sid_dest_name,
        role_name=DESTINATION_ROLE,
        interactive_mode=config.interactive_mode
    )


# ------------------------------------------------------
# Create and configure Amazon Managed Grafana workspace
# ------------------------------------------------------

# Create workspace
workspace_id, workspace_url = grafana_client.create_workspace(WORKSPACE_NAME, WORKSPACE_ROLE, config.interactive_mode)

# Store workspace url in config.json
config.set_workspace_url(workspace_url)

# Create workspace API key
workspace_api_key = grafana_client.create_workspace_api_key(workspace_id, WORKSPACE_API_KEY)

# Add timestream datasource
grafana_client.init_http_client(workspace_url, workspace_api_key)
datasource_uid = grafana_client.ws_add_datasource(DATASOURCE)

# Create dashboard from template
dashboard_id = grafana_client.ws_create_dashboard(
    template=Path(__file__).parent.joinpath('template', 'SidewalkGrafanaDashboard.json'),
    datasource_uid=datasource_uid
)

# Set the home dashboard
grafana_client.ws_set_home_dashboard(dashboard_id)

# Remove workspace API key
grafana_client.delete_workspace_api_key(workspace_id, WORKSPACE_API_KEY)


# -------------------------
# Print login info
# -------------------------
log_success('---------------------------------------------------------------')
log_success('Your Grafana workspace is ready. Log in using the link below:')
log_success(f'https://{workspace_url}')
log_success('---------------------------------------------------------------')


# -----------------------------------------------------
# Configure users with access to the Grafana workspace
# -----------------------------------------------------
if not (config.identity_store_id and config.identity_center_users):
    log_warn("IDENTITY_STORE_ID or IDENTITY_CENTER_USERS not given. Users will not be created.")
else:
    log_info(f'Creating users for the Grafana workspace...')
    if config.interactive_mode: confirm()

    # Create group in IAM Identity Center
    group_id = idstore_client.create_group(GROUP_NAME)

    # Create user in IAM Identity Center
    for idx, user in enumerate(config.identity_center_users):
        user.id = idstore_client.create_user(user)

    # Add users to the group
    for idx, user in enumerate(config.identity_center_users):
        idstore_client.add_user_to_group(user, group_id)
